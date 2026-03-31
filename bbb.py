import cv2
import time
import mediapipe as mp
from pyfirmata2 import Arduino
import threading

try:
    board = Arduino("COM3")
    servoX = board.get_pin('d:9:s')
    servoY = board.get_pin('d:10:s')
    led = board.get_pin('d:13:o')
    servoX.write(90)
    servoY.write(90)
    led.write(0)
except Exception as e:
    print(f"Arduino error: {e}")
    board = servoX = servoY = led = None

running = True


def move_servo(x, y):
    if servoX and servoY:
        x = 170 - (x - 10)
        servoX.write(x)
        servoY.write(y)


def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


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
                    if led:
                        led.write(1)
                    person_present = True

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
                if person_present:
                    print("Không có người — tắt chân 13, reset servo")
                    if led:
                        led.write(0)
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
        if led:
            led.write(0)
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
