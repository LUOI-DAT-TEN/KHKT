import cv2
import time
import mediapipe as mp
from pyfirmata2 import Arduino
import requests
import threading

PORT = "COM3"

INVERT_X = True
INVERT_Y = False

STATE_NO_PERSON = 0
STATE_LEVEL_1 = 1
STATE_LEVEL_2 = 2
STATE_LEVEL_3 = 3
STATE_CANCELLED = 4

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
except Exception as e:
    print(f"Không Ard: {e}")
    board = servoX = servoY = relays = None

state = STATE_NO_PERSON
start_time = None
alert_sent = False
waiting_for_cancel = False
cancel_command_received = False
person_detected = False
servo_x_target = 90
servo_y_target = 90
last_update_id = 0

state_lock = threading.Lock()
running = True


def send_telegram_alert(message):
    try:
        response = requests.post(
            f"https://api.telegram.org/bot7954299803:AAE1QSkQcEa6GPkTm5Su0X_j4Bft8TAhLu0/sendMessage",
            data={"chat_id": 8054440859, "text": message},
            timeout=5
        )
        if response.status_code == 200:
            print(f"✓ Tele: {message}")
            return True
        else:
            print(f"✗ Tele lỗi: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Không Telegram: {e}")
        return False


def check_telegram_messages():
    """Kiểm tra tin nhắn mới từ Telegram"""
    global last_update_id
    try:
        response = requests.get(
            f"https://api.telegram.org/bot7954299803:AAE1QSkQcEa6GPkTm5Su0X_j4Bft8TAhLu0/getUpdates",
            params={"offset": last_update_id + 1, "timeout": 0},
            timeout=2
        )

        if response.status_code == 200:
            data = response.json()
            if data.get("ok") and data.get("result"):
                messages = []
                for update in data["result"]:
                    last_update_id = update["update_id"]
                    if "message" in update and "text" in update["message"]:
                        text = update["message"]["text"].strip().lower()
                        messages.append(text)
                return messages
        return []
    except:
        return []


def set_relay(level):
    if not relays:
        return
    for i, relay in enumerate(relays):
        relay.write(1 if i == level else 0)


def move_servo(x, y):
    if servoX and servoY:
        if INVERT_X:
            x = 170 - (x - 10)
        if INVERT_Y:
            y = 170 - (y - 10)
        servoX.write(x)
        servoY.write(y)


def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def telegram_thread():
    global waiting_for_cancel, cancel_command_received, state, start_time
    global alert_sent, running

    wait_start_time = None
    WAIT_TIMEOUT = 5  # 15 giây


    while running:
        try:
            with state_lock:
                is_waiting = waiting_for_cancel
                current_state = state


            if is_waiting:
                if wait_start_time is None:
                    wait_start_time = time.time()


                elapsed = time.time() - wait_start_time
                if elapsed >= WAIT_TIMEOUT:
                    with state_lock:
                        waiting_for_cancel = False
                        cancel_command_received = False
                    wait_start_time = None
                else:
                    messages = check_telegram_messages()
                    if "huy" in messages:
                        with state_lock:
                            cancel_command_received = True
                            waiting_for_cancel = False
                            state = STATE_CANCELLED
                            start_time = None
                            alert_sent = False
                        wait_start_time = None
                        set_relay(-1)
                        move_servo(90, 90)
            else:
                wait_start_time = None

            time.sleep(1)
        except Exception as e:
            print(f"✗ Lỗi luồng Telegram: {e}")
            time.sleep(1)

def mediapipe_thread():
    global state, start_time, alert_sent, waiting_for_cancel
    global cancel_command_received, person_detected, running
    global servo_x_target, servo_y_target

    # Khởi tạo MediaPipe
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
    mp_draw = mp.solutions.drawing_utils

    # Khởi tạo camera
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    try:
        while running:
            ret, frame = cap.read()
            if not ret:
                print("✗ Không hình")
                break

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb)
            landmarks = results.pose_landmarks

            with state_lock:
                person_detected = landmarks is not None
                current_state = state
                is_waiting = waiting_for_cancel
                is_cancelled = cancel_command_received


            if current_state == STATE_CANCELLED:
                if person_detected:
                    mp_draw.draw_landmarks(frame, landmarks, mp_pose.POSE_CONNECTIONS)
                else:
                    with state_lock:
                        state = STATE_NO_PERSON
                        cancel_command_received = False
                        alert_sent = False
                        start_time = None

                cv2.imshow("Pose Tracking", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    running = False
                    break
                time.sleep(0.03)
                continue

            if person_detected:

                if current_state == STATE_NO_PERSON and not is_waiting:
                    print("1")
                    with state_lock:
                        state = STATE_LEVEL_1
                        start_time = time.time()

                    set_relay(0)


                    if not alert_sent:
                        if send_telegram_alert("Phát hiện người"):
                            with state_lock:
                                alert_sent = True
                                waiting_for_cancel = True
                                cancel_command_received = False


                if not is_waiting and not is_cancelled and current_state != STATE_NO_PERSON:
                    with state_lock:
                        if start_time:
                            elapsed = time.time() - start_time

                            if state == STATE_LEVEL_1 and elapsed >= 10:
                                print("2")
                                state = STATE_LEVEL_2
                                set_relay(1)

                            elif state == STATE_LEVEL_2 and elapsed >= 15:
                                print("3")
                                state = STATE_LEVEL_3
                                set_relay(2)


                    if current_state == STATE_LEVEL_3:
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
                if current_state != STATE_NO_PERSON and current_state != STATE_CANCELLED:
                    print("Không người")
                    with state_lock:
                        state = STATE_NO_PERSON
                        start_time = None
                        alert_sent = False
                        waiting_for_cancel = False
                        cancel_command_received = False
                    set_relay(-1)
                    move_servo(90, 90)

            cv2.imshow("Pose Tracking", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                running = False
                break

            time.sleep(0.03)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":

    telegram_worker = threading.Thread(target=telegram_thread, daemon=True)
    mediapipe_worker = threading.Thread(target=mediapipe_thread)

    telegram_worker.start()
    mediapipe_worker.start()

    try:
        mediapipe_worker.join()
    except KeyboardInterrupt:
        print("\n⏹️ Dừng ")
    finally:
        running = False
        if telegram_worker.is_alive():
            telegram_worker.join(timeout=2)
        if board:
            board.exit()
