"""Vision pipeline — chess piece recognition + ROS2 publish + Firebase write (entry point: ``ros2 run cobot2 object``).

ROS2 node:
    Subclass of ``rclpy.node.Node`` (V1-1 RESOLVED 2026-05-10, Phase 5 sub-phase A).
    Publishes recognized board state on ``/vision/board_state`` (``cobot2_interfaces/msg/BoardState``)
    using QoS RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1) so late-joining subscribers
    (e.g. ``main`` node, web UI via ``rosbridge_websocket``) receive the most recent value.
    Phase 5 sub-phase E (2026-05-10): Firebase dual-write 제거. ROS2 토픽이 단일
    publish 채널.

Pipeline (``VisionNode.run``):
    OpenCV ``VideoCapture(SOURCE)``
      → YOLO inference (foot-point per box)
      → grid polygon hit-test (``load_chess_grid``)
      → HSV V-channel threshold → piece color
      → ResNet18 classifier (6 chess piece classes) → ``WP``/``BR``/...
      → ``board_dict`` → ROS2 ``/vision/board_state`` publish

ROS2 parameters (declared in ``VisionNode.__init__``):
    - ``analyze_interval_sec``        (double, default 0.20) — minimum gap between YOLO+ResNet runs.
    - ``publish_min_interval_sec``    (double, default 0.20) — minimum gap between board_state publishes.
    - ``only_publish_on_change``      (bool, default True)  — skip publish/write if normalized board unchanged.
    - ``frame_id``                    (string, default ``chess_board``) — Header frame_id on published msgs.
                                       # verify needed: ``chess_board`` is project-defined; not REP-105 covered.

External Dependencies (env vars):
    - YOLO weights:   ``YOLO_PATH``  (``YOLO_MODEL_PATH`` env var, required).
    - ResNet weights: ``RESNET_PATH`` (``RESNET_MODEL_PATH`` env var, required).
    - Grid JSON:      ``GRID_PATH``  (``CHESS_GRID_PATH`` env var, required).
    - Camera:         ``SOURCE`` (``CAMERA_SOURCE`` env var, default ``3``).

Issues (Phase 1-1 doc Node 4):
    - V1-1 RESOLVED 2026-05-10 (Phase 5 sub-phase A): node now subclasses ``rclpy.node.Node`` and publishes
      ``/vision/board_state``. Firebase dual-write 잔존했으나 sub-phase E에서 일괄 제거 (2026-05-10).
    - V1-2 RESOLVED 2026-05-01: model paths env-ized.
    - V1-4 RESOLVED 2026-05-01: ``CAMERA_SOURCE`` env var.
    # verify needed V1-7: piece-color HSV V-channel thresholds (80, 105) — robustness across lighting.
    # verify needed V1-8: confirm ``CHESS_GRID_PATH`` env var points to valid ``chess_grid.json``.
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

from cobot2_interfaces.msg import BoardState


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
    """Load grid polygons from ``chess_grid.json``.

    Args:
        json_path: str — path to JSON file with ``{"A1": [[x,y],...], ...}``.

    Returns:
        dict[str, np.ndarray] — square name → polygon points reshaped to
        ``(-1, 1, 2)`` int32 (the layout ``cv2.pointPolygonTest`` expects),
        or ``None`` if the file does not exist.

    Notes:
        # verify needed V1-8: ``GRID_PATH`` is now ``CHESS_GRID_PATH`` env var (Phase 4 env-ize 2026-05-01).
        ``src/cobot2/config/chess_grid.json`` and ``src/cobot2/cobot2/chess_grid.json`` exist (byte-identical, Phase 1-4) —
        whether the env var points to the correct file is unverified.
    """
    if not os.path.exists(json_path):
        return None
    with open(json_path, "r") as f:
        data = json.load(f)
    return {sq: np.array(pts, dtype=np.int32).reshape((-1, 1, 2)) for sq, pts in data.items()}


def get_piece_color_improved(img, box):
    """Classify a detected piece as White / Black / Unknown via HSV V-channel threshold.

    Args:
        img: BGR frame (np.ndarray).
        box: ``(x1, y1, x2, y2)`` bounding box (any iterable of 4 numbers).

    Returns:
        ``"White"``, ``"Black"``, or ``"Unknown"`` (when the ROI is empty).

    Method:
        Crops the ROI to the band ``y ∈ [y1+0.2h, y1+0.4h]``, ``x ∈ [x1+0.42w, x1+0.58w]``
        (upper-middle strip of the bounding box), converts to HSV, and inspects the V channel.
        Returns ``"Black"`` if more than 30% of pixels have V < 80 *or* the median V < 105.

    Notes:
        # verify needed V1-7: HSV V-channel thresholds (80, 105) and the strip percentages
        (0.2, 0.4, 0.42, 0.58) are not validated for robustness across lighting conditions.
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
    """Run YOLO + grid mapping + color + ResNet on a single frame, return board dict.

    Args:
        frame:          BGR camera frame.
        yolo_model:     ``ultralytics.YOLO`` instance loaded from ``YOLO_PATH``.
        resnet_model:   ResNet18 with 6-class final layer (Pawn/Rook/Knight/Bishop/Queen/King).
        grid_polygons:  output of ``load_chess_grid`` or ``None`` (cells without grid → skipped).
        device:         torch device for ResNet inference.
        preprocess:     torchvision transform pipeline (Resize 224 → ToTensor → ImageNet Normalize).

    Returns:
        dict[str, str] — mapping square name (``"A1"``..``"H8"``) → piece code (``"WP"``/``"BR"``/...).
        Cells with no detected piece are absent from the dict.

    Side Effects:
        None (pure function over the frame).

    Logic:
        For each YOLO box: take the foot point ``((x1+x2)/2, y2)``, find the first grid polygon
        containing it (``cv2.pointPolygonTest``), classify color via ``get_piece_color_improved``,
        crop to RGB and run ResNet to pick a piece type, then write ``"{W|B}{P|R|N|B|Q|K}"``
        into the corresponding cell. YOLO is invoked with ``conf=0.5, iou=0.3``.
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
    if not d:
        return {}
    return {str(k): str(v) for k, v in sorted(d.items(), key=lambda x: x[0])}


class VisionNode(Node):
    """ROS2 node wrapping the camera + inference loop.

    Publishes ``BoardState`` on the relative topic ``vision/board_state`` (resolves
    to ``/vision/board_state`` at root namespace) with QoS RELIABLE + TRANSIENT_LOCAL
    + KEEP_LAST(1). Firebase write is performed in parallel during sub-phase A
    migration (removed in sub-phase E).

    Note on parameter dynamics:
        ``run()`` is a blocking OpenCV loop with no ``rclpy.spin()``. Parameters
        are read once in ``__init__`` and cached as instance attributes; runtime
        ``ros2 param set`` calls are accepted by the parameter server but will
        NOT take effect until the node restarts. ``add_on_set_parameters_callback``
        is intentionally not wired (sub-phase A scope). A future sub-phase that
        switches to a Timer + ``spin()`` model will re-enable dynamic params.
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
        # board_dict is expected to be the output of normalize_board_dict (already key-sorted),
        # but we re-sort defensively so callers can pass any dict-like board.
        msg = BoardState()
        msg.header.stamp = capture_stamp
        msg.header.frame_id = self.frame_id
        items = sorted(board_dict.items())
        msg.squares = [k for k, _ in items]
        msg.pieces = [v for _, v in items]
        msg.piece_count = len(items)
        self.board_state_pub.publish(msg)

    def _load_models(self):
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
            # Rule 7: 조용한 실패 방지. RuntimeError를 발생시켜 supervisor/launch가 관찰할 수 있도록.
            raise RuntimeError(f"Camera open failed (source={SOURCE})")

        self.get_logger().info(
            f"Vision running. Publishing on {BOARD_STATE_TOPIC}. Press Q to quit."
        )

        last_analyze_ts = 0.0
        last_publish_ts = 0.0
        last_sent_board = None

        while rclpy.ok():
            ret, frame = self.cap.read()
            # Rule 7: stamp = 측정 시점. cap.read() 직후 캡처.
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
