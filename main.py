# main.py

import threading
import numpy as np
import pybullet as p

from arm_sim import ArmSim
from gesture_module import run_gesture_loop


def main():
    arm = ArmSim(gui=True)

    state = {
        "gesture": "NONE",
        "cx": None,
        "cy": None,
        "size": None,
        "size0": None
    }

    alpha = 0.25  # smoothing factor

    def gesture_callback(gesture, cx, cy, size):
        state["gesture"] = gesture

        if cx is not None:
            state["cx"] = cx if state["cx"] is None else alpha * cx + (1 - alpha) * state["cx"]
            state["cy"] = cy if state["cy"] is None else alpha * cy + (1 - alpha) * state["cy"]
        else:
            state["cx"], state["cy"] = None, None

        state["size"] = size
        if state["size0"] is None and size is not None:
            state["size0"] = size

    threading.Thread(
        target=run_gesture_loop,
        args=(gesture_callback,),
        daemon=True
    ).start()

    print("Gesture control running (ESC to quit camera, Ctrl+C to exit).")

    try:
        while True:
            cx, cy = state["cx"], state["cy"]
            dx = dy = dz = 0.0

            # -------- Gripper --------
            if state["gesture"] == "FIST":
                arm.set_gripper(0.0)
            elif state["gesture"] == "OPEN":
                arm.set_gripper(1.0)

            # -------- XY & Z control --------
            if cx is not None and cy is not None:
                ox = cx - 0.5
                oy = cy - 0.5
                dead = 0.05

                if abs(ox) > dead:
                    dy = ox * 0.002
                if abs(oy) > dead:
                    dz = -oy * 0.002

            # -------- X (forward/back) from depth --------
            if state["size"] is not None and state["size0"] is not None:
                err = state["size"] - state["size0"]
                dx = np.clip(err, -0.02, 0.02) * 0.6

            arm.move_target(dx, dy, dz)
            arm.step()

    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
