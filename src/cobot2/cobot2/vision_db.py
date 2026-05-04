"""Vision pipeline — chess piece recognition + Firebase write (entry point: ``ros2 run cobot2 object``).

NOT a ROS2 node:
    This file does not import ``rclpy``, define a ``Node`` subclass, or call ``rclpy.init``. It is
    registered as a console script under ``cobot2`` but operates entirely outside the ROS2 graph.
    Communication with ``main.py`` flows through Firebase Realtime DB
    (``chess/board_state``) — Firebase is the message bus → ROS2 Rule 2 / Rule 7 violation (V1-1 CRITICAL).

Pipeline (``main()``, line 242):
    OpenCV ``VideoCapture(SOURCE)``
      → YOLO inference (foot-point per box)
      → grid polygon hit-test (``load_chess_grid``)
      → HSV V-channel threshold → piece color
      → ResNet18 classifier (6 chess piece classes) → ``WP``/``BR``/...
      → ``board_dict`` → Firebase ``chess/board_state``

Throttling:
    - ``ANALYZE_INTERVAL_SEC = 0.20`` (line 72) — minimum gap between YOLO+ResNet runs.
    - ``FIREBASE_UPDATE_MIN_INTERVAL_SEC = 0.20`` (line 73) — minimum gap between DB writes.
    - ``ONLY_UPDATE_ON_CHANGE`` (line 74) — skip write if normalized board dict unchanged.

External Dependencies:
    - YOLO weights:   ``YOLO_PATH``  (line 54) — ``YOLO_MODEL_PATH`` env var (required).
    - ResNet weights: ``RESNET_PATH`` (line 55) — ``RESNET_MODEL_PATH`` env var (required).
    - Grid JSON:      ``GRID_PATH``  (line 56) — ``CHESS_GRID_PATH`` env var (required).
    - Camera:         ``SOURCE`` (line 57) — ``CAMERA_SOURCE`` env var (default ``3``).
    - Firebase:       ``FIREBASE_SERVICE_ACCOUNT_PATH`` env var (line 66); ``FIREBASE_DATABASE_URL`` env var (line 67).

Issues (Phase 1-1 doc Node 4):
    - CRITICAL  V1-1: registered as a ROS2 entry point but does not participate in the ROS2 graph.
    - V1-2 RESOLVED 2026-05-01: ``YOLO_PATH``/``RESNET_PATH``/``GRID_PATH`` env-ized via ``YOLO_MODEL_PATH``/``RESNET_MODEL_PATH``/``CHESS_GRID_PATH`` env vars (lines 54-56).
    - V1-4 RESOLVED 2026-05-01: ``SOURCE`` env-ized via ``CAMERA_SOURCE`` env var (line 57, default 3).
    # verify needed V1-7: piece-color HSV V-channel thresholds (80, 105) — robustness across lighting.
    # verify needed V1-8: confirm ``CHESS_GRID_PATH`` env var points to valid ``chess_grid.json`` — content match with repo copy unverified.
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

import firebase_admin
from firebase_admin import credentials, db


# ================= [사용자 설정 구역] =================
YOLO_PATH = os.getenv("YOLO_MODEL_PATH")
RESNET_PATH = os.getenv("RESNET_MODEL_PATH")
GRID_PATH = os.getenv("CHESS_GRID_PATH")
SOURCE = int(os.getenv("CAMERA_SOURCE", "3"))

SAVE_DIR = "./captured_boards"
os.makedirs(SAVE_DIR, exist_ok=True)

CLASS_NAMES = ["Pawn", "Rook", "Knight", "Bishop", "Queen", "King"]
CLASS_ABBR = {"Pawn": "P", "Rook": "R", "Knight": "N", "Bishop": "B", "Queen": "Q", "King": "K"}

# ===== Firebase 설정 (env 주입) =====
FIREBASE_SERVICE_ACCOUNT_JSON = os.getenv("FIREBASE_SERVICE_ACCOUNT_PATH")
FIREBASE_DB_URL = os.getenv("FIREBASE_DATABASE_URL", "https://chess-43355-default-rtdb.asia-southeast1.firebasedatabase.app")
FIREBASE_DB_PATH = "chess/board_state"

# ===== 동작 옵션 =====
ANALYZE_INTERVAL_SEC = 0.20
FIREBASE_UPDATE_MIN_INTERVAL_SEC = 0.20
ONLY_UPDATE_ON_CHANGE = True

# 디버그 저장 (원하면 True)
SAVE_EACH_ANALYSIS_FRAME = False
# ====================================================


def now_iso_ms() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def init_firebase():
    if firebase_admin._apps:
        return
    if not FIREBASE_SERVICE_ACCOUNT_JSON:
        raise RuntimeError(
            "FIREBASE_SERVICE_ACCOUNT_PATH env var not set; cannot initialize Firebase"
        )
    cred = credentials.Certificate(FIREBASE_SERVICE_ACCOUNT_JSON)
    firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})


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


def update_firebase_board(ref, board_dict):
    payload = {
    "updated_at": now_iso_ms(),
    "piece_count": len(board_dict),
    "board": board_dict
    }
    ref.set(payload)



def main():
    init_firebase()
    ref = db.reference(FIREBASE_DB_PATH)

    yolo_model = YOLO(YOLO_PATH)
    resnet_model, device = load_resnet_model(RESNET_PATH)
    grid_polygons = load_chess_grid(GRID_PATH)

    preprocess = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

    cap = cv2.VideoCapture(SOURCE)
    if not cap.isOpened():
        print("Camera open failed.")
        return

    print("Vision running. Updates Firebase on each recognition. Press Q to quit.")

    last_analyze_ts = 0.0
    last_firebase_ts = 0.0
    last_sent_board = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        now_ts = time.time()
        display_frame = frame.copy()
        cv2.putText(display_frame, "RUNNING - Firebase Auto Update (Q: quit)", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        do_analyze = (now_ts - last_analyze_ts) >= ANALYZE_INTERVAL_SEC
        if do_analyze:
            last_analyze_ts = now_ts

            board_dict = analyze_frame(frame, yolo_model, resnet_model, grid_polygons, device, preprocess)
            board_norm = normalize_board_dict(board_dict)

            if SAVE_EACH_ANALYSIS_FRAME:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                cv2.imwrite(os.path.join(SAVE_DIR, f"frame_{ts}.jpg"), frame)

            should_send = True

            if ONLY_UPDATE_ON_CHANGE:
                if last_sent_board is not None and board_norm == last_sent_board:
                    should_send = False

            if should_send and (now_ts - last_firebase_ts) >= FIREBASE_UPDATE_MIN_INTERVAL_SEC:
                update_firebase_board(ref, board_norm)
                last_firebase_ts = now_ts
                last_sent_board = board_norm
                print(f"[DB UPDATED] squares={len(board_norm)} at {now_iso_ms()}")

        cv2.imshow("Chess Vision Tracker", display_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == ord("Q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
