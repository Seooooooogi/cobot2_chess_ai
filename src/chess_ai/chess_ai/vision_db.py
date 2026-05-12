"""체스 보드 비전 인식 노드 (entry point: ``ros2 run chess_ai object``).

OpenCV로 카메라를 읽어 YOLO + ResNet18 기반 분류를 수행하고, 인식된 보드 상태를
``vision/board_state`` topic (``chess_ai_interfaces/msg/BoardState``)으로 publish한다.

Pipeline:
    VideoCapture → YOLO detection (박스당 foot-point 추출)
        → grid polygon hit-test (chess_grid.json) → HSV V-channel piece-color 판정
        → ResNet18 6-class 분류 (Pawn/Rook/Knight/Bishop/Queen/King)
        → board_dict → BoardState publish.

Environment variables (필수):
    YOLO_MODEL_PATH (str): ultralytics YOLO weight 파일 경로.
    RESNET_MODEL_PATH (str): ResNet18 fine-tune weight 파일 경로.
    CHESS_GRID_PATH (str): 64-칸 grid polygon JSON 경로.
    CAMERA_SOURCE (int): ``cv2.VideoCapture`` index/URL. 기본 ``3``.

Note:
    Publish QoS는 RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)로 latched. 늦게 가입한
    subscriber (main, rosbridge_websocket 등)도 즉시 최신 보드를 받는다.
    ResNet 추론 device는 CUDA 가용 시 자동 GPU.
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

SAVE_EACH_ANALYSIS_FRAME = False  # True로 두면 매 분석 프레임을 jpg로 저장 (디버그용)

# 노드 코드에서는 상대 토픽명만 — 절대 경로 매핑·remap은 launch가 담당
BOARD_STATE_TOPIC = "vision/board_state"
# ====================================================


def now_iso_ms() -> str:
    """현재 시각을 millisecond 정밀도 ISO-8601 문자열로 반환한다."""
    return datetime.now().isoformat(timespec="milliseconds")


def load_chess_grid(json_path):
    """64-칸 grid polygon JSON을 ``cv2`` 호환 ndarray dict로 로드한다.

    Args:
        json_path (str): ``{"A1": [[x, y], ...], "A2": ...}`` 형식의 JSON 경로.

    Returns:
        dict[str, np.ndarray] | None: square 이름 → ``(-1, 1, 2)`` int32 폴리곤
            (``cv2.pointPolygonTest`` 요구 레이아웃). 파일이 없으면 ``None``.
    """
    if not os.path.exists(json_path):
        return None
    with open(json_path, "r") as f:
        data = json.load(f)
    return {sq: np.array(pts, dtype=np.int32).reshape((-1, 1, 2)) for sq, pts in data.items()}


def get_piece_color_improved(img, box):
    """Bounding box 내부 HSV V-channel로 White/Black/Unknown을 판정한다.

    바운딩 박스 상단 중앙 띠 (``y ∈ [y1+0.2h, y1+0.4h]``, ``x ∈ [x1+0.42w, x1+0.58w]``)를
    ROI로 잘라낸 뒤 V-channel을 검사한다. ``V < 80`` 픽셀이 30 %를 넘거나 V의 median이
    105 미만이면 ``Black``.

    Args:
        img (np.ndarray): BGR 프레임.
        box (Sequence[int]): ``(x1, y1, x2, y2)`` 바운딩 박스.

    Returns:
        str: ``"White"``, ``"Black"``, 또는 ROI가 비어있을 때 ``"Unknown"``.

    Note:
        V-channel 임계값(80/105)과 ROI 비율은 실험적으로 정한 상수다. 조명 변동에 대한
        robustness는 별도 검증이 필요하다.
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
    """ResNet18 (6-class final layer) weight를 로드해 eval 상태로 반환한다.

    Args:
        path (str): state_dict 경로. CPU로 로드한 뒤 device로 이동한다.

    Returns:
        tuple[torch.nn.Module, torch.device]: ``(model, device)``. CUDA 가용 시 GPU.
    """
    model = models.resnet18(weights=None)
    model.fc = nn.Linear(model.fc.in_features, len(CLASS_NAMES))
    model.load_state_dict(torch.load(path, map_location="cpu"))
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return model.to(device).eval(), device


def analyze_frame(frame, yolo_model, resnet_model, grid_polygons, device, preprocess):
    """단일 프레임에 detection → square 매핑 → color 판정 → classification을 수행한다.

    각 YOLO 박스의 foot point ``((x1+x2)/2, y2)``를 grid polygon과 hit-test 한 뒤,
    매칭된 square에 ``{색prefix}{기물약자}`` 코드를 기록한다. 박스가 보드 밖이거나
    crop이 비면 skip 한다.

    Args:
        frame (np.ndarray): BGR 카메라 프레임.
        yolo_model (ultralytics.YOLO): detection 모델. ``conf=0.5, iou=0.3``로 호출.
        resnet_model (torch.nn.Module): 6-class classifier.
        grid_polygons (dict[str, np.ndarray] | None): ``load_chess_grid`` 결과.
            ``None``이면 모든 박스가 skip 된다.
        device (torch.device): ResNet 추론 device.
        preprocess (torchvision.transforms.Compose):
            ``Resize(224) → ToTensor → ImageNet Normalize`` 파이프라인.

    Returns:
        dict[str, str]: square ("A1".."H8") → piece code ("WP"/"BR"/...). 검출되지 않은
            square는 dict에 포함되지 않는다.

    Note:
        Foot point를 사용하는 이유는 기물의 머리 좌표가 perspective에 따라 인접 square로
        밀려서 grid 매핑 오차를 키우기 때문이다.
    """
    results = yolo_model(frame, conf=0.5, iou=0.3, verbose=False)
    board_dict = {}

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            foot_point = ((x1 + x2) // 2, y2)

            square = None
            if grid_polygons:
                for sq, poly in grid_polygons.items():
                    # pointPolygonTest >= 0 → 경계 포함 내부
                    if cv2.pointPolygonTest(poly, foot_point, False) >= 0:
                        square = sq
                        break

            if not square:
                continue

            color = get_piece_color_improved(frame, [x1, y1, x2, y2])

            crop_bgr = frame[y1:y2, x1:x2]
            if crop_bgr.size == 0:
                continue

            crop = Image.fromarray(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB))
            input_tensor = preprocess(crop).unsqueeze(0).to(device)

            with torch.no_grad():
                outputs = resnet_model(input_tensor)
                _, pred_idx = torch.max(outputs, 1)
                abbr = CLASS_ABBR[CLASS_NAMES[pred_idx.item()]]

            color_prefix = "W" if color == "White" else "B"
            final_code = f"{color_prefix}{abbr}"
            board_dict[square] = final_code

    return board_dict


def normalize_board_dict(d):
    """key/value를 str로 강제하고 key 오름차순으로 정렬한다.

    정규화된 dict끼리만 ``==`` 비교로 'no change'를 판정할 수 있다.
    ``only_publish_on_change`` 분기와 메시지 직렬화가 본 함수의 출력을 소비한다.

    Args:
        d (dict): square → piece code. ``None``/빈 dict면 빈 dict를 반환한다.

    Returns:
        dict[str, str]: 정렬·문자열화된 dict.
    """
    if not d:
        return {}
    return {str(k): str(v) for k, v in sorted(d.items(), key=lambda x: x[0])}


class VisionNode(Node):
    """카메라 + 추론 루프를 wrapping 하는 ROS2 node.

    Publishes:
        vision/board_state (chess_ai_interfaces/msg/BoardState): 최신 보드 상태.
            QoS RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1) — 늦게 join한 subscriber도
            최신 값을 즉시 수신.

    Parameters:
        analyze_interval_sec (double, default 0.20): YOLO+ResNet 추론 간 최소 간격(초).
        publish_min_interval_sec (double, default 0.20): publish 간 최소 간격(초).
        only_publish_on_change (bool, default True): 정규화된 보드가 동일하면 publish 생략.
        frame_id (string, default ``"chess_board"``): 발행 메시지 ``Header.frame_id``.

    Warning:
        ``run()``은 OpenCV ``waitKey`` 기반 블로킹 루프이며 ``rclpy.spin()``을 호출하지
        않는다. parameter는 ``__init__``에서 1회 읽어 인스턴스 속성에 cache되므로,
        runtime ``ros2 param set``은 parameter server에만 반영되고 실제 동작에는 영향이
        없다. dynamic parameter가 필요하면 Timer + spin 모델로 전환할 것.
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
        """``BoardState`` 메시지를 구성해 발행한다.

        Args:
            board_dict (dict): square → piece code. 호출 전 정규화돼 있어도, 안전을 위해
                본 함수에서 다시 sort 한다.
            capture_stamp (builtin_interfaces.msg.Time): 프레임 캡처 시점.
                ``Header.stamp``로 사용된다 (측정 시점 기준).
        """
        msg = BoardState()
        msg.header.stamp = capture_stamp
        msg.header.frame_id = self.frame_id
        items = sorted(board_dict.items())
        msg.squares = [k for k, _ in items]
        msg.pieces = [v for _, v in items]
        msg.piece_count = len(items)
        self.board_state_pub.publish(msg)

    def _load_models(self):
        """필수 env를 검증하고 YOLO·ResNet·grid·preprocess를 인스턴스에 적재한다.

        Raises:
            RuntimeError: ``YOLO_MODEL_PATH`` / ``RESNET_MODEL_PATH`` / ``CHESS_GRID_PATH``
                env가 비어 있거나, 해당 경로 파일이 실제로 존재하지 않을 때.

        Note:
            env가 ``None``인 채로 ultralytics에 넘기면 silently ``"None" does not exist``
            트레이스를 내며, launch supervisor가 respawn 루프에 빠질 수 있어 fail-loud로
            차단한다.
        """
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
        """카메라를 열고 OpenCV 디스플레이 루프를 돌면서 board_state를 발행한다.

        ``cap.read()`` 직후 ``capture_stamp``를 캡처해 publish 메시지의 ``Header.stamp``로
        쓴다 (발행 시점이 아니라 측정 시점). 추론 throttle과 publish throttle은 각각
        독립된 ``analyze_interval_sec``, ``publish_min_interval_sec``으로 제어된다.
        ``Q`` 키 또는 ``rclpy.ok() == False``로 종료한다.

        Raises:
            RuntimeError: ``cv2.VideoCapture(SOURCE)`` 가 열리지 않을 때.

        Note:
            ``cv2.imshow`` 헤더 문구는 사용자 표시 텍스트일 뿐이며 동작에 영향이 없다.
        """
        self._load_models()

        self.cap = cv2.VideoCapture(SOURCE)
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera open failed (source={SOURCE})")

        self.get_logger().info(
            f"Vision running. Publishing on {BOARD_STATE_TOPIC}. Press Q to quit."
        )

        last_analyze_ts = 0.0
        last_publish_ts = 0.0
        last_sent_board = None

        while rclpy.ok():
            ret, frame = self.cap.read()
            # stamp = 측정 시점 — cap.read() 직후 캡처해야 시간 동기화가 의미 있음
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

            # 매 프레임마다 YOLO+ResNet을 돌리면 CPU/GPU 과부하 — analyze_interval_sec으로 throttle
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

                # only_publish_on_change: 보드가 동일하면 publish 생략 (TRANSIENT_LOCAL이라
                # 신규 subscriber는 어차피 마지막 값을 받는다)
                should_send = True
                if self.only_publish_on_change and last_sent_board is not None and board_norm == last_sent_board:
                    should_send = False

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
        """카메라·OpenCV window를 해제한다."""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    """Entry point: ``VisionNode`` 생성 → ``run`` → ``cleanup`` → ``shutdown``.

    SIGINT는 ``KeyboardInterrupt``로 받아 정상 종료한다. ``rclpy.shutdown()``은
    ``rclpy.ok()`` 가드 후 호출 — supervisor가 이미 shutdown한 context를 재호출하면
    ``RCLError``가 발생한다.
    """
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
