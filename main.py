# main.py

import threading
import pybullet as p
import numpy as np

from arm_sim import ArmSim
from gesture_module import run_gesture_loop


def main():
    arm = ArmSim(gui=True)

    # Shared state between camera thread and sim thread
    state = {
        "gesture": "NONE",
        "cx_raw": None,
        "cy_raw": None,
        "cx": None,   # smoothed
        "cy": None
    }

    alpha = 0.2  # smoothing factor for EMA (0=very smooth, 1=no smoothing)

    def gesture_callback(gesture, cx, cy):
        # Raw values from MediaPipe
        state["gesture"] = gesture
        state["cx_raw"] = cx
        state["cy_raw"] = cy

        # Exponential moving average for smooth control
        if cx is not None and cy is not None:
            if state["cx"] is None:
                # first frame: just copy
                state["cx"] = cx
                state["cy"] = cy
            else:
                state["cx"] = alpha * cx + (1 - alpha) * state["cx"]
                state["cy"] = alpha * cy + (1 - alpha) * state["cy"]
        else:
            # no hand detected
            state["cx"] = None
            state["cy"] = None

    # Run webcam + gesture detection in background
    t = threading.Thread(target=run_gesture_loop, args=(gesture_callback,))
    t.daemon = True
    t.start()

    print("Gesture control running. Ctrl+C to exit.")

    try:
        while True:
            gesture = state["gesture"]
            cx = state["cx"]
            cy = state["cy"]

            # ----- Gripper from gesture -----
            if gesture == "FIST":
                arm.set_gripper(0.0)   # close
            elif gesture == "OPEN":
                arm.set_gripper(1.0)   # open

            # ----- Smooth arm motion from hand position -----
            dx = dy = dz = 0.0

            if cx is not None and cy is not None:
                # deviation from center
                offset_x = cx - 0.5  # left/right
                offset_y = cy - 0.5  # up/down

                # deadzone to avoid jitter around center
                deadzone = 0.05
                if abs(offset_x) < deadzone:
                    offset_x = 0.0
                if abs(offset_y) < deadzone:
                    offset_y = 0.0

                # scale to velocities
                # smaller value -> slower, smoother motion
                scale = 0.002

                dy = scale * offset_x          # left/right
                dz = -scale * offset_y         # up/down (invert y)

                # you can also control X (forward/back) with gesture if you want:
                # dx = scale * something

            # apply incremental move
            if dx or dy or dz:
                arm.move_target(dx=dx, dy=dy, dz=dz)

            # step the simulation
            arm.step()

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
