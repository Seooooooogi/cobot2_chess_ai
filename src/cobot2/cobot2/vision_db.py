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
YOLO_PATH = "/home/kyb/cobot_ws/src/cobot2/cobot2/train_pt/best.pt"
RESNET_PATH = "/home/kyb/cobot_ws/src/cobot2/cobot2/train_pt/classifier.pt"
GRID_PATH = "/home/kyb/cobot_ws/src/cobot2/config/chess_grid.json"
SOURCE = 3

SAVE_DIR = "./captured_boards"
os.makedirs(SAVE_DIR, exist_ok=True)

CLASS_NAMES = ["Pawn", "Rook", "Knight", "Bishop", "Queen", "King"]
CLASS_ABBR = {"Pawn": "P", "Rook": "R", "Knight": "N", "Bishop": "B", "Queen": "Q", "King": "K"}

# ===== Firebase 설정 (사용자가 제공한 값으로 고정) =====
FIREBASE_SERVICE_ACCOUNT_JSON = "/home/kyb/cobot_ws/src/cobot2/config/kybfirebase.json"
FIREBASE_DB_URL = "https://chess-43355-default-rtdb.asia-southeast1.firebasedatabase.app"
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
    cred = credentials.Certificate(FIREBASE_SERVICE_ACCOUNT_JSON)
    firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})


def load_chess_grid(json_path):
    if not os.path.exists(json_path):
        return None
    with open(json_path, "r") as f:
        data = json.load(f)
    return {sq: np.array(pts, dtype=np.int32).reshape((-1, 1, 2)) for sq, pts in data.items()}


def get_piece_color_improved(img, box):
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
