import cv2
import time
import mediapipe as mp
import threading

arduino_connected = False
board = servoX = servoY = led = None

try:
    from pyfirmata2 import Arduino
    board = Arduino("COM3")
    time.sleep(2)
    servoX = board.get_pin('d:9:s')
    servoY = board.get_pin('d:10:s')
    led = board.get_pin('d:13:o')
    servoX.write(90)
    servoY.write(90)
    led.write(0)
    arduino_connected = True
    print("Arduino kết nối thành công")
except Exception as e:
    print(f"Không kết nối được Arduino: {e}")

running = True
last_servo_time = 0
SERVO_INTERVAL = 0.15


def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))


def move_servo(x, y):
    global last_servo_time
    now = time.time()
    if now - last_servo_time < SERVO_INTERVAL:
        return
    last_servo_time = now

    # Ép sang int, clamp 0-180
    x = clamp(int(round(float(x))), 0, 180)
    y = clamp(int(round(float(y))), 0, 180)

    print(f"Servo → X: {x}, Y: {y}")

    if servoX and servoY:
        servoX.write(x)
        servoY.write(y)


def set_pin13(on: bool):
    if led:
        led.write(1 if on else 0)
    print(f"Chân 13: {'BẬT' if on else 'TẮT'}")


def map_value(value, in_min, in_max, out_min, out_max):
    result = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return clamp(int(round(result)), min(out_min, out_max), max(out_min, out_max))


def mediapipe_thread():
    global running

    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
    mp_draw = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    person_present = False

    try:
        while running:
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb)
            landmarks = results.pose_landmarks

            if landmarks:
                if not person_present:
                    print("Có người — bật chân 13")
                    set_pin13(True)
                    person_present = True

                try:
                    nose = landmarks.landmark[mp_pose.PoseLandmark.NOSE]
                    hip = landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP]

                    cx = (nose.x + hip.x) / 2  # 0.0 - 1.0
                    cy = (nose.y + hip.y) / 2  # 0.0 - 1.0

                    # Map trực tiếp từ 0.0-1.0 sang 0-180, clamp chặt
                    sx = clamp(int(round((1.0 - cx) * 180)), 0, 180)
                    sy = clamp(int(round(cy * 180)), 0, 180)

                    move_servo(sx, sy)
                except Exception as e:
                    print(f"Lỗi tính toán: {e}")

                mp_draw.draw_landmarks(frame, landmarks, mp_pose.POSE_CONNECTIONS)

            else:
                if person_present:
                    print("Không có người — tắt chân 13, reset servo")
                    set_pin13(False)
                    move_servo(90, 90)
                    person_present = False

            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                running = False
                break

            time.sleep(0.03)

    except KeyboardInterrupt:
        pass
    finally:
        set_pin13(False)
        move_servo(90, 90)
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    worker = threading.Thread(target=mediapipe_thread)
    worker.start()
    try:
        worker.join()
    except KeyboardInterrupt:
        running = False
    finally:
        if board:
            board.exit()
