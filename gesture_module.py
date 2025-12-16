# gesture_module.py

import cv2
import mediapipe as mp
import numpy as np

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils


def classify_gesture(landmarks):
    wrist = np.array([landmarks[0].x, landmarks[0].y])
    tips = [4, 8, 12, 16, 20]

    dists = []
    for i in tips:
        tip = np.array([landmarks[i].x, landmarks[i].y])
        dists.append(np.linalg.norm(tip - wrist))

    avg = np.mean(dists)
    if avg < 0.08:
        return "FIST"
    elif avg > 0.16:
        return "OPEN"
    return "UNKNOWN"


def hand_center(landmarks):
    xs = [lm.x for lm in landmarks]
    ys = [lm.y for lm in landmarks]
    return float(np.mean(xs)), float(np.mean(ys))


def hand_size(landmarks):
    xs = [lm.x for lm in landmarks]
    ys = [lm.y for lm in landmarks]
    return float((max(xs) - min(xs)) * (max(ys) - min(ys)))


def run_gesture_loop(callback):
    cap = cv2.VideoCapture(0)

    with mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.7
    ) as hands:

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = hands.process(rgb)

            gesture, cx, cy, size = "NONE", None, None, None

            if res.multi_hand_landmarks:
                lm = res.multi_hand_landmarks[0]
                gesture = classify_gesture(lm.landmark)
                cx, cy = hand_center(lm.landmark)
                size = hand_size(lm.landmark)

                mp_draw.draw_landmarks(frame, lm, mp_hands.HAND_CONNECTIONS)

            callback(gesture, cx, cy, size)

            cv2.putText(frame, f"Gesture: {gesture}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Gesture Control", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

    cap.release()
    cv2.destroyAllWindows()
