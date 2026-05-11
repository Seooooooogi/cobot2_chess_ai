"""Vision 파이프라인 — 체스 기물 인식 + ROS2 publish (entry point: ``ros2 run chess_ai object``).

ROS2 node:
    ``rclpy.node.Node`` 서브클래스 (V1-1 RESOLVED 2026-05-10, Phase 5 sub-phase A).
    인식된 보드 상태를 ``/vision/board_state`` (``chess_ai_interfaces/msg/BoardState``)에
    QoS RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)로 publish — 늦게 가입한 subscriber
    (``main`` 노드, ``rosbridge_websocket`` 경유 Web UI 등)도 최신 값을 즉시 수신.
    Phase 5 sub-phase E (2026-05-10): Firebase dual-write 제거. ROS2 토픽이 단일
    publish 채널.

파이프라인 (``VisionNode.run``):
    OpenCV ``VideoCapture(SOURCE)``
      → YOLO inference (박스당 foot-point 추출)
      → grid polygon hit-test (``load_chess_grid``)
      → HSV V-channel threshold → 기물 색상 판정
      → ResNet18 classifier (6 클래스: Pawn/Rook/Knight/Bishop/Queen/King) → ``WP``/``BR``/...
      → ``board_dict`` → ROS2 ``/vision/board_state`` publish

ROS2 parameters (``VisionNode.__init__``에서 declare):
    - ``analyze_interval_sec``        (double, default 0.20) — YOLO+ResNet 추론 간 최소 간격(초).
    - ``publish_min_interval_sec``    (double, default 0.20) — board_state publish 간 최소 간격(초).
    - ``only_publish_on_change``      (bool, default True)  — 정규화된 보드가 동일하면 publish 생략.
    - ``frame_id``                    (string, default ``chess_board``) — publish 메시지 Header.frame_id.
                                       # verify needed: ``chess_board``는 프로젝트 정의 — REP-105 covered 아님.

외부 의존성 (env vars):
    - YOLO weights:   ``YOLO_PATH``  (``YOLO_MODEL_PATH`` env var, 필수).
    - ResNet weights: ``RESNET_PATH`` (``RESNET_MODEL_PATH`` env var, 필수).
    - Grid JSON:      ``GRID_PATH``  (``CHESS_GRID_PATH`` env var, 필수).
    - Camera:         ``SOURCE`` (``CAMERA_SOURCE`` env var, default ``3``).

Issues (Phase 1-1 doc Node 4):
    - V1-1 RESOLVED 2026-05-10 (Phase 5 sub-phase A): ``rclpy.node.Node`` 서브클래스화 +
      ``/vision/board_state`` publish. Firebase dual-write 잔존 → sub-phase E에서 일괄 제거 (2026-05-10).
    - V1-2 RESOLVED 2026-05-01: model paths env-ized.
    - V1-4 RESOLVED 2026-05-01: ``CAMERA_SOURCE`` env var 도입.
    # verify needed V1-7: piece-color HSV V-channel 임계값 (80, 105) — 조명 변동에 대한 robustness 미확인.
    # verify needed V1-8: ``CHESS_GRID_PATH`` env var가 올바른 ``chess_grid.json``을 가리키는지 확인.
"""

import cv2
import torch
import torch.nn as nn
from torchvision import models, transforms
from ultralytics import YOLO
from PIL import Image
import numpy as np
import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from chess_ai_interfaces.msg import BoardState


# ================= [사용자 설정 구역] =================
YOLO_PATH = os.getenv("YOLO_MODEL_PATH")
RESNET_PATH = os.getenv("RESNET_MODEL_PATH")
GRID_PATH = os.getenv("CHESS_GRID_PATH")
SOURCE = int(os.getenv("CAMERA_SOURCE", "3"))

SAVE_DIR = "./captured_boards"

CLASS_NAMES = ["Pawn", "Rook", "Knight", "Bishop", "Queen", "King"]
CLASS_ABBR = {"Pawn": "P", "Rook": "R", "Knight": "N", "Bishop": "B", "Queen": "Q", "King": "K"}

# 디버그 저장 (원하면 True)
SAVE_EACH_ANALYSIS_FRAME = False

# ===== ROS2 토픽 (Rule 5: 노드 코드는 상대 경로만; 절대 경로 매핑은 launch에서) =====
BOARD_STATE_TOPIC = "vision/board_state"
# ====================================================


def now_iso_ms() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def load_chess_grid(json_path):
    """``chess_grid.json``에서 격자 폴리곤을 로드한다.

    Args:
        json_path: str — ``{"A1": [[x,y],...], ...}`` 형식의 JSON 파일 경로.

    Returns:
        dict[str, np.ndarray] — 칸 이름 → ``(-1, 1, 2)`` int32 폴리곤 점 배열
        (``cv2.pointPolygonTest``가 요구하는 레이아웃). 파일 부재 시 ``None``.

    Notes:
        # verify needed V1-8: ``GRID_PATH``는 현재 ``CHESS_GRID_PATH`` env var (Phase 4 env-ize 2026-05-01).
        ``src/chess_ai/config/chess_grid.json``와 ``src/chess_ai/chess_ai/chess_grid.json``이 동시 존재
        (byte-identical, Phase 1-4) — env var가 올바른 파일을 가리키는지 미검증.
    """
    if not os.path.exists(json_path):
        return None
    with open(json_path, "r") as f:
        data = json.load(f)
    return {sq: np.array(pts, dtype=np.int32).reshape((-1, 1, 2)) for sq, pts in data.items()}


def get_piece_color_improved(img, box):
    """검출된 기물의 색상을 HSV V채널 임계값으로 White / Black / Unknown 분류한다.

    Args:
        img: BGR 프레임 (np.ndarray).
        box: ``(x1, y1, x2, y2)`` 바운딩 박스 (길이 4 iterable).

    Returns:
        ``"White"``, ``"Black"``, 또는 ROI가 비었을 때 ``"Unknown"``.

    방법:
        바운딩 박스의 상단 중앙 띠 ``y ∈ [y1+0.2h, y1+0.4h]``, ``x ∈ [x1+0.42w, x1+0.58w]``
        를 ROI로 잘라 HSV로 변환 후 V채널을 검사한다.
        V < 80 픽셀 비율이 30%를 넘거나 V 중앙값이 105 미만이면 ``"Black"``.

    Notes:
        # verify needed V1-7: HSV V채널 임계값(80, 105)과 ROI 비율(0.2, 0.4, 0.42, 0.58)이
        조명 변동에 대한 robustness 측면에서 검증되지 않음.
    """
    x1, y1, x2, y2 = map(int, box)
    w, h = x2 - x1, y2 - y1

    roi_y1, roi_y2 = y1 + int(h * 0.2), y1 + int(h * 0.4)
    roi_x1, roi_x2 = x1 + int(w * 0.42), x1 + int(w * 0.58)

    roi = img[max(0, roi_y1):min(img.shape[0], roi_y2),
              max(0, roi_x1):min(img.shape[1], roi_x2)]

    if roi.size == 0:
        return "Unknown"

    v = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)[:, :, 2]
    is_black = (np.sum(v < 80) / v.size) > 0.3 or np.median(v) < 105
    return "Black" if is_black else "White"


def load_resnet_model(path):
    model = models.resnet18(weights=None)
    model.fc = nn.Linear(model.fc.in_features, len(CLASS_NAMES))
    model.load_state_dict(torch.load(path, map_location="cpu"))
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return model.to(device).eval(), device


def analyze_frame(frame, yolo_model, resnet_model, grid_polygons, device, preprocess):
    """단일 프레임에 YOLO + grid 매핑 + 색상 판정 + ResNet을 수행해 보드 dict을 반환한다.

    Args:
        frame:          BGR 카메라 프레임.
        yolo_model:     ``YOLO_PATH``에서 로드한 ``ultralytics.YOLO`` 인스턴스.
        resnet_model:   ResNet18 (6-class final layer: Pawn/Rook/Knight/Bishop/Queen/King).
        grid_polygons:  ``load_chess_grid``의 결과 또는 ``None`` (격자 없으면 해당 칸 skip).
        device:         ResNet 추론용 torch device.
        preprocess:     torchvision transform 파이프라인 (Resize 224 → ToTensor → ImageNet Normalize).

    Returns:
        dict[str, str] — 칸 이름 (``"A1"``..``"H8"``) → 기물 코드 (``"WP"``/``"BR"``/...) 매핑.
        검출되지 않은 칸은 dict에서 누락된다.

    Side Effects:
        없음 (프레임에 대한 순수 함수).

    동작:
        각 YOLO 박스마다 foot point ``((x1+x2)/2, y2)``를 잡아 ``cv2.pointPolygonTest``로
        포함하는 첫 grid 폴리곤을 찾고, ``get_piece_color_improved``로 색상 판정 후
        RGB crop을 ResNet에 통과시켜 기물 종류를 결정한다.
        결과는 ``"{W|B}{P|R|N|B|Q|K}"`` 코드로 해당 칸에 기록.
        YOLO 인자: ``conf=0.5, iou=0.3``.
    """
    # YOLO 추론: confidence 0.5 이상, IoU 0.3 이하(중복 박스 억제)
    results = yolo_model(frame, conf=0.5, iou=0.3, verbose=False)
    board_dict = {}

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # 체스 말의 발 지점(foot point): 바운딩 박스 하단 중앙.
            # 말의 머리가 아닌 발로 어느 칸에 있는지 판별해야 격자 매핑 오차가 줄어든다.
            foot_point = ((x1 + x2) // 2, y2)

            # 발 지점이 어느 체스판 격자 폴리곤 안에 있는지 순차 탐색.
            # pointPolygonTest >= 0 : 경계 포함 내부. 격자를 찾지 못하면 skip.
            square = None
            if grid_polygons:
                for sq, poly in grid_polygons.items():
                    if cv2.pointPolygonTest(poly, foot_point, False) >= 0:
                        square = sq
                        break

            if not square:
                continue  # 보드 밖 검출 무시

            # HSV V채널 임계값으로 흰/검 구분
            color = get_piece_color_improved(frame, [x1, y1, x2, y2])

            # ResNet 입력용 크롭 (바운딩 박스 전체, BGR → RGB)
            crop_bgr = frame[y1:y2, x1:x2]
            if crop_bgr.size == 0:
                continue

            crop = Image.fromarray(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB))
            # preprocess: Resize(224,224) → ToTensor → ImageNet Normalize
            input_tensor = preprocess(crop).unsqueeze(0).to(device)

            with torch.no_grad():
                outputs = resnet_model(input_tensor)
                _, pred_idx = torch.max(outputs, 1)
                abbr = CLASS_ABBR[CLASS_NAMES[pred_idx.item()]]  # 'P','R','N','B','Q','K'

            # 최종 코드: "W" + 기물약자 or "B" + 기물약자 (예: "WP", "BK")
            color_prefix = "W" if color == "White" else "B"
            final_code = f"{color_prefix}{abbr}"
            board_dict[square] = final_code

    return board_dict


def normalize_board_dict(d):
    """보드 딕셔너리를 정규화한다 — 키/값을 문자열로, 알파벳 순 정렬.

    정규화된 dict 끼리만 == 비교로 '변경 없음' 판단이 가능하다.
    only_publish_on_change 로직과 ROS2 메시지 직렬화 모두 이 함수를 통과한 dict를 사용.
    """
    if not d:
        return {}
    return {str(k): str(v) for k, v in sorted(d.items(), key=lambda x: x[0])}


class VisionNode(Node):
    """카메라 + 추론 루프를 감싸는 ROS2 노드.

    상대 토픽 ``vision/board_state`` (root namespace에서 ``/vision/board_state``로 풀림)에
    QoS RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)로 ``BoardState``를 publish한다.
    sub-phase A 마이그레이션 기간엔 Firebase write 병행 → sub-phase E에서 제거됨.

    Parameter 동적 변경 주의:
        ``run()``은 ``rclpy.spin()`` 없이 OpenCV 루프로 블로킹된다. Parameter는
        ``__init__``에서 1회 읽어 인스턴스 속성에 캐시되며, 런타임 ``ros2 param set``은
        parameter server엔 반영되나 노드 재시작 전엔 실제 동작에 적용되지 않는다.
        ``add_on_set_parameters_callback``은 의도적으로 미연결 (sub-phase A 범위).
        향후 Timer + ``spin()`` 모델로 전환되는 sub-phase에서 dynamic param 재활성.
    """

    def __init__(self):
        super().__init__("vision_db")

        self.declare_parameter("analyze_interval_sec", 0.20)
        self.declare_parameter("publish_min_interval_sec", 0.20)
        self.declare_parameter("only_publish_on_change", True)
        self.declare_parameter("frame_id", "chess_board")

        self.analyze_interval_sec = float(
            self.get_parameter("analyze_interval_sec").value
        )
        self.publish_min_interval_sec = float(
            self.get_parameter("publish_min_interval_sec").value
        )
        self.only_publish_on_change = bool(
            self.get_parameter("only_publish_on_change").value
        )
        self.frame_id = str(self.get_parameter("frame_id").value)

        board_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.board_state_pub = self.create_publisher(
            BoardState, BOARD_STATE_TOPIC, board_state_qos
        )

        self.cap = None
        self.yolo_model = None
        self.resnet_model = None
        self.device = None
        self.grid_polygons = None
        self.preprocess = None

    def _publish_board_state(self, board_dict, capture_stamp):
        # board_dict는 normalize_board_dict의 출력(이미 키 정렬됨)을 받지만,
        # 임의 dict-like 입력도 받을 수 있도록 방어적으로 다시 정렬한다.
        msg = BoardState()
        msg.header.stamp = capture_stamp
        msg.header.frame_id = self.frame_id
        items = sorted(board_dict.items())
        msg.squares = [k for k, _ in items]
        msg.pieces = [v for _, v in items]
        msg.piece_count = len(items)
        self.board_state_pub.publish(msg)

    def _load_models(self):
        # Rule 7 (fail-loud): 필수 env 미설정 시 명시 RuntimeError. None 통과 시
        # ultralytics가 'None' does not exist 트레이스 후 launch respawn 무한 루프
        # (PB-5 회귀 방지).
        missing = [
            name for name, value in (
                ("YOLO_MODEL_PATH", YOLO_PATH),
                ("RESNET_MODEL_PATH", RESNET_PATH),
                ("CHESS_GRID_PATH", GRID_PATH),
            ) if not value
        ]
        if missing:
            raise RuntimeError(
                f"vision_db missing required env vars: {missing}. "
                "Source src/chess_ai/.env (or set them via launch 'env=' / shell) before launch. "
                "See .env.example for keys."
            )
        not_found = [
            (name, value) for name, value in (
                ("YOLO_MODEL_PATH", YOLO_PATH),
                ("RESNET_MODEL_PATH", RESNET_PATH),
                ("CHESS_GRID_PATH", GRID_PATH),
            ) if not os.path.exists(value)
        ]
        if not_found:
            raise RuntimeError(
                f"vision_db env paths point to non-existent files: {not_found}. "
                "Verify model + grid files exist."
            )

        self.yolo_model = YOLO(YOLO_PATH)
        self.resnet_model, self.device = load_resnet_model(RESNET_PATH)
        self.grid_polygons = load_chess_grid(GRID_PATH)
        self.preprocess = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

    def run(self):
        self._load_models()

        self.cap = cv2.VideoCapture(SOURCE)
        if not self.cap.isOpened():
            # Rule 7 (silent failure 방지): RuntimeError로 supervisor/launch가 관찰 가능하게.
            raise RuntimeError(f"Camera open failed (source={SOURCE})")

        self.get_logger().info(
            f"Vision running. Publishing on {BOARD_STATE_TOPIC}. Press Q to quit."
        )

        last_analyze_ts = 0.0
        last_publish_ts = 0.0
        last_sent_board = None

        while rclpy.ok():
            ret, frame = self.cap.read()
            # Rule 7 (stamp = 측정 시점): cap.read() 직후 stamp 캡처.
            capture_stamp = self.get_clock().now().to_msg()
            if not ret:
                break

            now_ts = time.time()
            display_frame = frame.copy()
            cv2.putText(
                display_frame,
                "RUNNING - ROS2 + Firebase Auto Update (Q: quit)",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            # 속도 제한 (rate-limiting): analyze_interval_sec 마다 1회 추론.
            # 매 프레임마다 YOLO+ResNet을 돌리면 CPU/GPU 과부하 — 일정 간격으로 throttle.
            if (now_ts - last_analyze_ts) >= self.analyze_interval_sec:
                last_analyze_ts = now_ts

                board_dict = analyze_frame(
                    frame,
                    self.yolo_model,
                    self.resnet_model,
                    self.grid_polygons,
                    self.device,
                    self.preprocess,
                )
                board_norm = normalize_board_dict(board_dict)

                if SAVE_EACH_ANALYSIS_FRAME:
                    os.makedirs(SAVE_DIR, exist_ok=True)
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    cv2.imwrite(os.path.join(SAVE_DIR, f"frame_{ts}.jpg"), frame)

                # only_publish_on_change: 보드 상태가 이전과 동일하면 발행 생략.
                # TRANSIENT_LOCAL latched이므로 가입자는 구독 시점에 최신 값을 받음.
                should_send = True
                if self.only_publish_on_change and last_sent_board is not None and board_norm == last_sent_board:
                    should_send = False

                # 추가 publish 속도 제한 (publish_min_interval_sec).
                # analyze_interval과 독립적으로 조절 가능.
                if should_send and (now_ts - last_publish_ts) >= self.publish_min_interval_sec:
                    self._publish_board_state(board_norm, capture_stamp)
                    last_publish_ts = now_ts
                    last_sent_board = board_norm
                    self.get_logger().info(
                        f"[PUBLISHED] squares={len(board_norm)} at {now_iso_ms()}"
                    )

            cv2.imshow("Chess Vision Tracker", display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == ord("Q"):
                break

    def cleanup(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
