# AMR YOLOv11 Tracking

Dự án nhận diện đối tượng/biển báo bằng YOLOv11 để điều khiển robot hoặc xe tự hành qua camera, Python và ESP32.

## Chức năng

- Nhận diện biển báo giao thông và gửi lệnh điều khiển qua serial.
- Bám theo vật thể theo thời gian thực với điều khiển PD.
- Chuyển chế độ tự động giữa tracking và search trên ESP32.

## Cấu trúc

- `dectect_Object.py`: nhận diện biển báo và gửi lệnh sang ESP32 qua serial.
- `bamVatThe_autonomous.py`: bám vật thể, điều khiển tốc độ trái/phải qua HTTP.
- `esp32_lxtp/esp32_lxtp.ino`: firmware ESP32 cho motor L298N.
- `datasheet8object.pt`, `objectTracking1.pt`: mô hình YOLO đã huấn luyện.

## Yêu cầu

- Python 3.9+.
- OpenCV, PyTorch, Ultralytics, Requests, PySerial.
- Camera USB hoặc DroidCam.
- ESP32 kết nối với mạch L298N.

## Cách chạy

1. Cài thư viện Python cần thiết.
2. Cập nhật `MODEL_PATH`, `SERIAL_PORT` hoặc địa chỉ `ESP32_CTRL_URL` / `ESP32_MODE_URL` nếu cần.
3. Chạy một trong hai file Python phù hợp với chế độ bạn muốn dùng.
4. Nạp sketch trong `esp32_lxtp/esp32_lxtp.ino` lên ESP32.

## Lưu ý

- Kiểm tra đúng cổng serial và địa chỉ IP của ESP32 trước khi chạy.
- Đảm bảo mô hình `.pt` nằm đúng thư mục gốc của project.
