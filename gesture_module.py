import cv2
import mediapipe as mp
import numpy as np

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils


def classify_gesture(landmarks):
    wrist = np.array([landmarks[0].x, landmarks[0].y])
    tips_idx = [4, 8, 12, 16, 20]

    dists = []
    for i in tips_idx:
        tip = np.array([landmarks[i].x, landmarks[i].y])
        dists.append(np.linalg.norm(tip - wrist))

    avg_dist = np.mean(dists)

    if avg_dist < 0.08:
        return "FIST"
    elif avg_dist > 0.16:
        return "OPEN"
    else:
        return "UNKNOWN"


def get_hand_center(landmarks):
    xs = [lm.x for lm in landmarks]
    ys = [lm.y for lm in landmarks]
    cx = float(np.mean(xs))
    cy = float(np.mean(ys))
    return cx, cy


def run_gesture_loop(callback):
    cap = cv2.VideoCapture(0)  # if laptop webcam fails, try 1 instead of 0

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    with mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.7
    ) as hands:

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(frame_rgb)

            gesture = "NONE"
            cx, cy = None, None

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                gesture = classify_gesture(hand_landmarks.landmark)
                cx, cy = get_hand_center(hand_landmarks.landmark)

                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS
                )

            callback(gesture, cx, cy)

            cv2.putText(frame, f"Gesture: {gesture}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Hand Control", frame)

            if cv2.waitKey(1) & 0xFF == 27:
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    def test_callback(g, x, y):
        print(f"Gesture={g}, cx={x}, cy={y}")

    run_gesture_loop(test_callback)
