import cv2
import time
import mediapipe as mp
from pyfirmata2 import Arduino
import requests
PORT = "COM3"

INVERT_X = True
INVERT_Y = False

# Các trạng thái
STATE_NO_PERSON = 0
STATE_LEVEL_1 = 1
STATE_LEVEL_2 = 2
STATE_LEVEL_3 = 3

print("🔌 Đang khởi động hệ thống...")

try:
    board = Arduino(PORT)

    servoX = board.get_pin('d:9:s')
    servoY = board.get_pin('d:10:s')

    relays = [
        board.get_pin('d:2:o'),
        board.get_pin('d:3:o'),
        board.get_pin('d:4:o')
    ]

    servoX.write(90)
    servoY.write(90)
    print(f"✅ Arduino kết nối thành công tại {PORT}")

except Exception as e:
    print(f"❌ Không thể kết nối Arduino: {e}")
    board = servoX = servoY = relays = None

mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
print("📷 Camera đã sẵn sàng.\n")

def send_telegram_alert(message="🚨 Phát hiện người trong khu vực giám sát!"):
    try:
        response = requests.post(
            f"https://api.telegram.org/bot7954299803:AAE1QSkQcEa6GPkTm5Su0X_j4Bft8TAhLu0/sendMessage",
            data={"chat_id": "8054440859", "text": message},
            timeout=5
        )
        if response.status_code == 200:
            print("📨 Đã gửi cảnh báo Telegram.")
        else:
            print(f"❌ Telegram lỗi: {response.text}")
    except Exception as e:
        print(f"⚠️ Không thể gửi Telegram: {e}")

def set_relay(level):
    if not relays:
        return
    for i, relay in enumerate(relays):
        relay.write(1 if i == level else 0)

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def move_servo(x, y):
    if servoX and servoY:
        if INVERT_X:
            x = 170 - (x - 10)
        if INVERT_Y:
            y = 170 - (y - 10)
        servoX.write(x)
        servoY.write(y)


print("🚀 Hệ thống giám sát bắt đầu (nhấn ESC để thoát).")

state = STATE_NO_PERSON
start_time = None
alert_sent = False

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Không thể đọc khung hình.")
            break


        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb)
        landmarks = results.pose_landmarks
        person_detected = landmarks is not None

        if person_detected:
            if state == STATE_NO_PERSON:
                print("👤 Phát hiện người → Cấp 1")
                state = STATE_LEVEL_1
                start_time = time.time()
                set_relay(0)
                if not alert_sent:
                    send_telegram_alert()
                    alert_sent = True

            elapsed = time.time() - start_time

            if state == STATE_LEVEL_1 and elapsed >= 5:
                print("⏳ >30s → Cấp 2")
                state = STATE_LEVEL_2
                set_relay(1)

            elif state == STATE_LEVEL_2 and elapsed >= 10:
                print("🚨 >60s → Cấp 3 (theo dõi)")
                state = STATE_LEVEL_3
                set_relay(2)

            if state == STATE_LEVEL_3:
                try:
                    nose = landmarks.landmark[mp_pose.PoseLandmark.NOSE]
                    hip = landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP]
                    cx = int((nose.x + hip.x) / 2 * 640)
                    cy = int((nose.y + hip.y) / 2 * 480)
                    sx = map_value(cx, 0, 640, 170, 10)
                    sy = map_value(cy, 0, 480, 170, 10)
                    move_servo(sx, sy)
                except:
                    pass

            mp_draw.draw_landmarks(frame, landmarks, mp_pose.POSE_CONNECTIONS)

        else:
            if state != STATE_NO_PERSON:
                print("⭕ Không còn người → Reset hệ thống.")
                state = STATE_NO_PERSON
                start_time = None
                alert_sent = False
                set_relay(-1)
                move_servo(90, 90)

        cv2.imshow("📡 Giám sát người (MediaPipe Pose)", frame)

        # Nhấn ESC để thoát
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("🧭 Nhấn ESC → Thoát hệ thống.")
            break

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n🛑 Dừng thủ công.")
finally:
    print("🔻 Dọn tài nguyên...")
    if board:
        board.exit()
    cap.release()
    cv2.destroyAllWindows()
    print("✅ Hoàn tất thoát hệ thống.")