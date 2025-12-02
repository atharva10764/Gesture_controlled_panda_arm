# arm_sim.py

import pybullet as p
import pybullet_data
import time
import numpy as np


class ArmSim:
    def __init__(self, gui=True):
        if gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # ---------- World ----------
        self.plane_id = p.loadURDF("plane.urdf")

        # A small cube to pick and place
        # (start on the ground in front of the robot)
        self.cube_start_pos = [0.6, 0.0, 0.05]
        self.cube_id = p.loadURDF(
            "cube_small.urdf",
            self.cube_start_pos,
            p.getQuaternionFromEuler([0, 0, 0])
        )

        # Extra friction so it doesn't slide away too easily
        p.changeDynamics(self.cube_id, -1, lateralFriction=1.0)

        # ---------- Panda arm with gripper ----------
        start_pos = [0, 0, 0]
        start_orn = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF(
            "franka_panda/panda.urdf",
            start_pos,
            start_orn,
            useFixedBase=True
        )

        self.num_joints = p.getNumJoints(self.robot_id)

        # 7 arm joints
        self.arm_joints = list(range(7))

        # Gripper finger joints (left & right)
        self.finger_joints = [9, 10]

        # End-effector link index
        self.ee_link_index = 11

        # Joint limits and ranges (from Franka specs / PyBullet examples)
        self.lower_limits = [
            -2.8973, -1.7628, -2.8973,
            -3.0718, -2.8973, -0.0175, -2.8973
        ]
        self.upper_limits = [
            2.8973, 1.7628, 2.8973,
            -0.0698, 2.8973, 3.7525, 2.8973
        ]
        self.joint_ranges = [
            5.8, 3.5, 5.8,
            3.0, 5.8, 3.8, 5.8
        ]
        # Comfortable “home” configuration
        self.rest_poses = [
            0.0, -0.3, 0.0,
            -2.2, 0.0, 2.0, 0.8
        ]

        # Reset to rest pose
        for j, q in zip(self.arm_joints, self.rest_poses):
            p.resetJointState(self.robot_id, j, q)

        # Start with EE above the cube area
        self.target_pos = np.array([0.5, 0.0, 0.4], dtype=float)
        self.target_orn = p.getQuaternionFromEuler([0, 0, 0])

        # Gripper: 0 = closed, 1 = open
        self.gripper_open_amount = 1.0

        # Turn off default motors so we fully control the joints
        for j in range(self.num_joints):
            p.setJointMotorControl2(
                self.robot_id,
                j,
                p.VELOCITY_CONTROL,
                force=0
            )

        p.setTimeStep(1.0 / 240.0)

    # ---------- Gripper control ----------
    def set_gripper(self, open_amount: float):
        open_amount = float(np.clip(open_amount, 0.0, 1.0))
        self.gripper_open_amount = open_amount

        finger_max = 0.04  # ~4 cm opening
        target_pos = finger_max * open_amount

        for j in self.finger_joints:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_pos,
                force=50
            )

    # ---------- IK + step ----------
    def step(self):
        # Solve IK for 7 arm joints
        joint_poses = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_link_index,
            targetPosition=self.target_pos,
            targetOrientation=self.target_orn,
            lowerLimits=self.lower_limits,
            upperLimits=self.upper_limits,
            jointRanges=self.joint_ranges,
            restPoses=self.rest_poses
        )

        for i, j in enumerate(self.arm_joints):
            if i >= len(joint_poses):
                break
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                force=200
            )

        # Apply gripper position
        self.set_gripper(self.gripper_open_amount)

        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    # ---------- Target motion ----------
    def move_target(self, dx=0.0, dy=0.0, dz=0.0):
        self.target_pos += np.array([dx, dy, dz], dtype=float)

        # clamp workspace
        self.target_pos[0] = float(np.clip(self.target_pos[0], 0.2, 0.8))
        self.target_pos[1] = float(np.clip(self.target_pos[1], -0.4, 0.4))
        self.target_pos[2] = float(np.clip(self.target_pos[2], 0.1, 0.8))

    def reset_cube(self):
        """Reset cube back to start position (for repeated demos)."""
        p.resetBasePositionAndOrientation(
            self.cube_id,
            self.cube_start_pos,
            p.getQuaternionFromEuler([0, 0, 0])
        )


if __name__ == "__main__":
    # Quick sanity test
    arm = ArmSim(gui=True)
    # open gripper
    arm.set_gripper(1.0)
    for _ in range(240):
        arm.step()
    # close gripper
    arm.set_gripper(0.0)
    for _ in range(240):
        arm.step()
