import cv2
import requests
import numpy as np
from ultralytics import YOLO
import time
import math

# ================= CONFIG =================
MODEL_PATH = "objectTracking1.pt"
DROIDCAM_INDEX = 0

# ESP32 endpoints
ESP32_CTRL_URL = "http://192.168.2.220/control"
ESP32_MODE_URL = "http://192.168.2.220/mode"

CONF_THRESH = 0.8
CONF_THRESH_LOW = 0.8  # Ngưỡng thấp để phát hiện sớm
BASE_SPEED = 80  # Giảm tốc độ cơ bản
MAX_SPEED = 255
STOP_SPEED = 0
DISPLAY_SIZE = (640, 480)
LOST_LIMIT = 5

# ===== PD CONTROL (Centerline Tracking) =====
KP = 120              # Proportional gain - giảm để quay chậm hơn
KD = 40               # Derivative gain - giảm dao động/overshooting
ANGLE_DEADZONE = 0.08  # Vùng chết góc (radians, ~4.5°)

# ===== DISTANCE ESTIMATION =====
OBJECT_REAL_WIDTH = 20   # Chiều rộng thực vật thể (cm) - điều chỉnh theo vật thể
FOCAL_LENGTH = 500       # Focal length ước lượng - cần calibrate
MIN_DISTANCE = 30        # Khoảng cách tối thiểu (cm)
MAX_DISTANCE = 200       # Khoảng cách tối đa (cm)
CLOSE_DISTANCE = 50      # Khoảng cách gần → giảm tốc
FAR_DISTANCE = 150       # Khoảng cách xa → tăng tốc

# Mode switching với cooldown
NO_OBJECT_LIMIT = 45      # Số frame không thấy object → chuyển SEARCH
MODE_COOLDOWN = 2.0       # Thời gian khóa mode (giây)

# ===== MODES =====
MODE_TRACKING = "tracking"
MODE_SEARCH = "search"

# ================= INIT =================
print("▶️ Loading YOLO model...")
model = YOLO(MODEL_PATH)

print(f"▶️ Connecting to DroidCam (Index {DROIDCAM_INDEX})...")
cap = cv2.VideoCapture(DROIDCAM_INDEX)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("❌ Failed to open DroidCam!")
    exit()

print("✅ DroidCam connected!")

last_cmd = None
lost_count = 0
current_mode = MODE_TRACKING
no_object_count = 0
last_mode_switch = 0  # Thời điểm chuyển mode gần nhất

# ===== PD Control State =====
prev_angle = 0.0
prev_time = time.time()

# ================= UTILS =================
def mapfloat(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def can_switch_mode():
    """Kiểm tra đã hết cooldown chưa"""
    return (time.time() - last_mode_switch) >= MODE_COOLDOWN

def get_cooldown_remaining():
    """Lấy thời gian cooldown còn lại"""
    remaining = MODE_COOLDOWN - (time.time() - last_mode_switch)
    return max(0, remaining)

# ================= CENTERLINE TRACKING =================
def calculate_angle(obj_center_x, obj_center_y, camera_x, camera_y):
    """
    Tính góc lệch giữa đường tâm (object → camera) và đường thẳng đứng.
    Góc > 0: Vật ở bên PHẢI → cần rẽ phải
    Góc < 0: Vật ở bên TRÁI → cần rẽ trái
    """
    dx = obj_center_x - camera_x  # Độ lệch ngang
    dy = camera_y - obj_center_y  # Khoảng cách dọc (luôn dương)
    if dy <= 0:
        dy = 1  # Tránh chia cho 0
    return math.atan2(dx, dy)  # Góc trong radians

def estimate_distance(box_width, frame_width):
    """
    Ước lượng khoảng cách từ camera đến vật thể dựa trên kích thước box.
    Công thức: distance = (real_width * focal_length) / pixel_width
    """
    if box_width <= 0:
        return MAX_DISTANCE
    # Điều chỉnh theo tỷ lệ frame
    normalized_width = box_width * (640 / frame_width)
    distance = (OBJECT_REAL_WIDTH * FOCAL_LENGTH) / normalized_width
    return max(MIN_DISTANCE, min(MAX_DISTANCE, distance))

def pd_control(angle, dt):
    """
    Điều khiển PD dựa trên góc lệch.
    Returns: Giá trị điều khiển (-255 đến 255)
    """
    global prev_angle
    if dt <= 0:
        dt = 0.033  # Default ~30fps
    
    # Proportional: Tỷ lệ với góc lệch
    p_term = KP * angle
    
    # Derivative: Tốc độ thay đổi góc (giảm dao động)
    d_term = KD * (angle - prev_angle) / dt
    
    prev_angle = angle
    
    # Giới hạn output
    output = p_term + d_term
    return max(-MAX_SPEED, min(MAX_SPEED, output))

def get_speed_factor(distance):
    """
    Tính hệ số tốc độ dựa trên khoảng cách.
    Gần → chậm, Xa → nhanh
    """
    if distance <= CLOSE_DISTANCE:
        return 0.5  # Chậm lại khi gần
    elif distance >= FAR_DISTANCE:
        return 1.2  # Tăng tốc khi xa
    else:
        # Linear interpolation
        return 0.5 + 0.7 * (distance - CLOSE_DISTANCE) / (FAR_DISTANCE - CLOSE_DISTANCE)

# ================= HTTP =================
def send_speed(left, right):
    """Gửi lệnh điều khiển motor (chỉ dùng trong TRACKING mode)"""
    global last_cmd
    cmd = (left, right)
    if cmd == last_cmd:
        return True
    try:
        requests.post(
            ESP32_CTRL_URL,
            json={"left": int(left), "right": int(right)},
            timeout=0.3  # Giảm timeout
        )
        print(f"📤 L:{left:4d} R:{right:4d}")
        last_cmd = cmd
        return True
    except Exception as e:
        print(f"❌ Motor error: {e}")
        return False

def set_esp_mode(mode):
    """Gửi lệnh chuyển mode cho ESP32"""
    global last_mode_switch
    try:
        response = requests.post(
            ESP32_MODE_URL,
            json={"mode": mode},
            timeout=1.0
        )
        last_mode_switch = time.time()  # Cập nhật thời điểm chuyển mode
        print(f"🔄 ESP32 mode → {mode.upper()} (cooldown {MODE_COOLDOWN}s)")
        return True
    except Exception as e:
        print(f"❌ Mode switch error: {e}")
        return False

# ================= YOLO =================
def get_best_box(result):
    """Trả về box tốt nhất và confidence của nó"""
    if result.boxes is None:
        return None, 0.0
    best = None
    best_conf = 0.0
    for b in result.boxes:
        conf = float(b.conf[0])
        if conf > best_conf:
            best_conf = conf
            best = b.xyxy[0].tolist()
    return best, best_conf

# ================= MAIN =================
print("▶️ Object Following + Autonomous Obstacle Avoidance START")
print(f"   Mode: {current_mode.upper()}")
print(f"   Cooldown: {MODE_COOLDOWN}s giữa các lần chuyển mode")

# Đảm bảo ESP32 bắt đầu ở TRACKING mode
set_esp_mode(MODE_TRACKING)

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        lost_count += 1
        print(f"⚠️  Frame lost ({lost_count}/{LOST_LIMIT})")
        if lost_count > LOST_LIMIT:
            print("❌ Lost signal - STOP")
            send_speed(STOP_SPEED, STOP_SPEED)
        continue
    
    lost_count = 0
    h, w = frame.shape[:2]
    result = model(frame, verbose=False)[0]
    box, confidence = get_best_box(result)
    cx = 0.0
    cooldown_remaining = get_cooldown_remaining()
    
    # ================= MODE SWITCHING LOGIC =================
    # Phát hiện vật với độ tin cậy cao
    high_conf_detected = box is not None and confidence >= CONF_THRESH
    # Phát hiện vật với độ tin cậy thấp (để chuẩn bị chuyển mode)
    low_conf_detected = box is not None and confidence >= CONF_THRESH_LOW
    
    if high_conf_detected:
        # Phát hiện rõ ràng → reset counter
        no_object_count = 0
        
        # Chuyển từ SEARCH → TRACKING: BỎ QUA COOLDOWN (ưu tiên tracking)
        if current_mode == MODE_SEARCH:
            print(f"👁️ Object detected with {confidence:.0%} confidence!")
            current_mode = MODE_TRACKING
            if set_esp_mode(MODE_TRACKING):
                print("✅ Switched to TRACKING mode (bypassed cooldown)")
            last_cmd = None  # Reset để gửi lệnh mới
        
    elif low_conf_detected:
        # Phát hiện mờ → giữ nguyên mode hiện tại, không tăng counter
        # Điều này giúp tránh chuyển sang SEARCH khi object chỉ tạm thời mất
        pass
        
    else:
        # Không thấy gì → tăng counter
        no_object_count += 1
    
    # Chuyển sang SEARCH nếu mất object quá lâu (cần cooldown)
    if current_mode == MODE_TRACKING:
        if no_object_count >= NO_OBJECT_LIMIT and can_switch_mode():
            current_mode = MODE_SEARCH
            set_esp_mode(MODE_SEARCH)
    
    # ================= TRACKING MODE =================
    if current_mode == MODE_TRACKING:
        if high_conf_detected:
            x1, y1, x2, y2 = box
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            
            # ===== CENTERLINE TRACKING =====
            # Tâm vật thể
            obj_cx = (x1 + x2) // 2
            obj_cy = (y1 + y2) // 2
            
            # Điểm camera (giữa đáy frame)
            cam_x = w // 2
            cam_y = h
            
            # Tính góc lệch
            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time
            
            angle = calculate_angle(obj_cx, obj_cy, cam_x, cam_y)
            angle_deg = math.degrees(angle)
            
            # Ước lượng khoảng cách
            box_width = x2 - x1
            distance = estimate_distance(box_width, w)
            speed_factor = get_speed_factor(distance)
            
            # PD Control output
            turn_output = pd_control(angle, dt)
            
            # Tính tốc độ motor
            base = int(BASE_SPEED * speed_factor)
            
            if abs(angle) < ANGLE_DEADZONE:
                # Trong vùng chết → đi thẳng
                left_speed = base
                right_speed = base
                action = "→ FORWARD"
                line_color = (0, 255, 0)  # Green - centered
            else:
                # Áp dụng PD control
                turn = int(turn_output)
                # Đảo ngược: angle > 0 (vật bên phải) → tăng left, giảm right → quay phải
                left_speed = base + turn
                right_speed = base - turn
                
                # Giới hạn tốc độ
                left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
                right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
                
                if angle < 0:
                    action = f"← LEFT ({angle_deg:.1f}°)"
                else:
                    action = f"→ RIGHT ({angle_deg:.1f}°)"
                line_color = (0, 0, 255)  # Red - off center
            
            send_speed(left_speed, right_speed)
            
            # ===== VISUALIZATION =====
            # Vẽ bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Vẽ đường tâm từ vật thể đến camera
            cv2.line(frame, (obj_cx, obj_cy), (cam_x, cam_y), line_color, 3)
            
            # Vẽ điểm tâm vật thể
            cv2.circle(frame, (obj_cx, obj_cy), 8, (255, 0, 255), -1)
            
            # Vẽ điểm camera
            cv2.circle(frame, (cam_x, cam_y - 10), 8, (255, 255, 0), -1)
            
            status = f"🟢 TRACKING ({confidence:.0%})"
            extra_info = f"Angle: {angle_deg:+.1f}° | Dist: {distance:.0f}cm"
            
        elif low_conf_detected:
            # Thấy mờ → đi chậm/dừng, chờ xác nhận
            x1, y1, x2, y2 = box
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)  # Yellow box
            status = f"🟡 LOW CONF ({confidence:.0%})"
            action = "⊠ SLOWING"
            extra_info = ""
            left_speed = BASE_SPEED // 2
            right_speed = BASE_SPEED // 2
            send_speed(left_speed, right_speed)
            
        else:
            # Chờ chuyển mode
            if cooldown_remaining > 0:
                status = f"🟡 WAIT ({no_object_count}/{NO_OBJECT_LIMIT}) CD:{cooldown_remaining:.1f}s"
            else:
                status = f"🟡 SEARCHING... ({no_object_count}/{NO_OBJECT_LIMIT})"
            action = "⊠ WAITING"
            extra_info = ""
            left_speed = STOP_SPEED
            right_speed = STOP_SPEED
            send_speed(left_speed, right_speed)
    
    # ================= SEARCH MODE =================
    else:
        status = "🔵 SEARCH (ESP32 autonomous)"
        action = "🤖 ESP32 đang tự lái"
        extra_info = ""
        left_speed = 0
        right_speed = 0
        
        # Hiển thị nếu phát hiện object (chuẩn bị chuyển mode)
        if low_conf_detected:
            x1, y1, x2, y2 = box
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            if cooldown_remaining > 0:
                action = f"👁️ Detected! CD:{cooldown_remaining:.1f}s"
            else:
                action = "👁️ Detected! Switching..."
    
    # ================= DISPLAY =================
    if current_mode == MODE_TRACKING:
        mode_color = (0, 255, 0)  # Green
    else:
        mode_color = (255, 165, 0)  # Orange
    
    # Đường chia đôi màn hình (reference line)
    cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 0), 2)
    
    # Info overlay
    cv2.putText(frame, f"MODE: {current_mode.upper()}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)
    cv2.putText(frame, status, (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, action, (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    if current_mode == MODE_TRACKING:
        cv2.putText(frame, f"L:{left_speed:4d} R:{right_speed:4d}", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if extra_info:
            cv2.putText(frame, extra_info, (10, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
    
    # Cooldown indicator
    if cooldown_remaining > 0:
        cv2.putText(frame, f"COOLDOWN: {cooldown_remaining:.1f}s", (w - 200, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    cv2.imshow("TRACKING + AUTONOMOUS SEARCH", cv2.resize(frame, DISPLAY_SIZE))
    
    if cv2.waitKey(1) == 27:  # ESC
        print("🛑 Stopping...")
        set_esp_mode(MODE_TRACKING)
        send_speed(STOP_SPEED, STOP_SPEED)
        break

# ================= CLEANUP =================
cap.release()
cv2.destroyAllWindows()
print("✅ DONE")
