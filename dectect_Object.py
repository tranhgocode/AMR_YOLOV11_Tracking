import cv2
import time
import torch
import serial
from ultralytics import YOLO

# ================= CONFIG =================
MODEL_PATH = "datasheet8object.pt"
CAM_INDEX = 0
CONF_THRESH = 0.88
SERIAL_PORT = "COM8"
SERIAL_BAUD = 115200
MIN_BOX_AREA_RATIO = 0.05  # Tỷ lệ diện tích bounding box so với frame (5%)
MAX_BOX_AREA_RATIO = 0.40  # Giới hạn trên để tránh biển báo quá to

# ================= COOLDOWN SETTINGS =================
COMMAND_COOLDOWN = {
    "STOP": 0.3,
    "FORWARD": 1.2,
    "LEFT": 2.5,
    "RIGHT": 2.5,
    "TURN_AROUND": 2.5,
    "PEDESTRIAN": 1.5,
    "SLOW_DOWN": 0.5,
    "SPEED_RESUME": 0.5
}

# ================= YOLO =================
CLASS_NAMES = [
    "giam_toc", "het_gioi_han_toc_do", "di_thang",
    "re_trai", "nguoi_qua_duong",
    "re_phai", "quay_dau", "stop"
]

CLASS_TO_CMD = {
    "giam_toc": "SLOW_DOWN",
    "het_gioi_han_toc_do": "SPEED_RESUME",
    "di_thang": "FORWARD",
    "re_trai": "LEFT",
    "nguoi_qua_duong": "PEDESTRIAN",
    "re_phai": "RIGHT",
    "quay_dau": "TURN_AROUND",
    "stop": "STOP"
}

PRIORITY = {
    "stop": 100,
    "nguoi_qua_duong": 90,
    "giam_toc": 80,
    "re_trai": 70,
    "re_phai": 70,
    "quay_dau": 60,
    "di_thang": 50,
    "het_gioi_han_toc_do": 40
}

ONE_SHOT_CMDS = {"LEFT", "RIGHT", "TURN_AROUND"}

# ================= SERIAL =================
try:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    time.sleep(2)
    print(" Serial connected:", SERIAL_PORT)
except Exception as e:
    print(" KHÔNG MỞ ĐƯỢC SERIAL:", e)
    exit()

# ================= STATE =================
last_cmd = None
last_cmd_time = {}
current_state = "STOP"

# ================= LOCK SYSTEM =================
command_lock = False
lock_until = 0
LOCK_TIME = 2.0  # khóa 2 giây sau khi gửi lệnh

def send_cmd(cmd):
    global last_cmd, current_state
    now = time.time()

    # Cooldown
    if cmd in last_cmd_time:
        if now - last_cmd_time[cmd] < COMMAND_COOLDOWN.get(cmd, 0.5):
            return False

    try:
        ser.write(f"{cmd}\n".encode())
        time.sleep(0.05)
        last_cmd = cmd
        last_cmd_time[cmd] = now
        current_state = cmd
        print(f"✓ SEND → {cmd}")
        return True
    except Exception as e:
        print(f"❌ Gửi lỗi: {e}")
        return False

# ================= MODEL =================
device = "cuda" if torch.cuda.is_available() else "cpu"
model = YOLO(MODEL_PATH).to(device)

# ================= CAMERA =================
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("❌ KHÔNG MỞ ĐƯỢC CAMERA")
    exit()

print("🎥 Camera opened – Serial active")

# ================= LOOP =================
while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ KHÔNG ĐỌC ĐƯỢC FRAME")
        break

    result = model(frame, conf=CONF_THRESH, verbose=False)[0]

    best_score = 0
    best_class = None
    best_area_ratio = 0

    if result.boxes:
        scores = result.boxes.conf.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        boxes_xyxy = result.boxes.xyxy.cpu().numpy()
        
        frame_height, frame_width = frame.shape[:2]
        frame_area = frame_width * frame_height

        for i in range(len(scores)):
            score = scores[i]
            if score < CONF_THRESH:
                continue

            # Lấy tọa độ bounding box
            x1, y1, x2, y2 = boxes_xyxy[i]
            
            # Tính diện tích bounding box
            box_width = x2 - x1
            box_height = y2 - y1
            box_area = box_width * box_height
            
            # Tính tỷ lệ diện tích so với frame
            area_ratio = box_area / frame_area
            
            # Kiểm tra điều kiện kích thước
            if area_ratio < MIN_BOX_AREA_RATIO or area_ratio > MAX_BOX_AREA_RATIO:
                continue  # Bỏ qua nếu biển báo quá nhỏ hoặc quá to
            
            name = CLASS_NAMES[classes[i]]
            p = PRIORITY[name] * score * (1 + area_ratio)  # Ưu tiên cả kích thước
            
            if p > best_score:
                best_score = p
                best_class = name
                best_area_ratio = area_ratio

    # ========================= LOCK LOGIC =========================
    now = time.time()

    if command_lock:
        if now >= lock_until:
            command_lock = False
            print(" Mở khóa – sẵn sàng xử lý biển báo mới")
        else:
            # đang khóa → không gửi lệnh
            pass
    else:
        if best_class:
            cmd = CLASS_TO_CMD[best_class]
            
            # Gửi lệnh chỉ khi bounding box đủ lớn
            if best_area_ratio >= MIN_BOX_AREA_RATIO:
                send_cmd(cmd)

                # khóa lại 2 giây
                command_lock = True
                lock_until = now + LOCK_TIME
                print(f" Đã gửi lệnh {cmd} → KHÓA {LOCK_TIME}s (Area ratio: {best_area_ratio:.3f})")

    # ========================= DISPLAY =========================
    frame = result.plot()
    
    # Vẽ thông tin kích thước
    if best_class and best_area_ratio > 0:
        size_text = f"Size: {best_area_ratio:.3f}"
        cv2.putText(frame, size_text, (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 0), 2)
    
    cv2.putText(frame, f"Current: {current_state}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Hiển thị ngưỡng kích thước
    cv2.putText(frame, f"Min Size: {MIN_BOX_AREA_RATIO}", (10, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 255), 1)
    
    if best_class:
        cv2.putText(frame, f"Detected: {best_class} ({best_score:.2f})", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow("YOLO Traffic Sign", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ================= CLEANUP =================
cap.release()
ser.close()
cv2.destroyAllWindows()
print(" Done")