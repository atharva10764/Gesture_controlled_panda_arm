# arm_sim.py

import pybullet as p
import pybullet_data
import numpy as np
import time


class ArmSim:
    def __init__(self, gui=True):
        if gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)

        # ---------- Ground ----------
        p.loadURDF("plane.urdf")

        # ---------- Cube (object to pick) ----------
        self.cube_start_pos = [0.6, 0.0, 0.03]
        self.cube_id = p.loadURDF(
            "cube_small.urdf",
            self.cube_start_pos,
            p.getQuaternionFromEuler([0, 0, 0])
        )
        p.changeDynamics(self.cube_id, -1, lateralFriction=1.0)

        # ---------- Panda arm ----------
        self.robot_id = p.loadURDF(
            "franka_panda/panda.urdf",
            [0, 0, 0],
            p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True
        )

        self.arm_joints = list(range(7))
        self.finger_joints = [9, 10]
        self.ee_link_index = 11

        # Joint limits (Franka Panda)
        self.lower_limits = [
            -2.8973, -1.7628, -2.8973,
            -3.0718, -2.8973, -0.0175, -2.8973
        ]
        self.upper_limits = [
            2.8973, 1.7628, 2.8973,
            -0.0698, 2.8973, 3.7525, 2.8973
        ]
        self.joint_ranges = [u - l for u, l in zip(self.upper_limits, self.lower_limits)]
        self.rest_poses = [0, -0.3, 0, -2.2, 0, 2.0, 0.8]

        for j, q in zip(self.arm_joints, self.rest_poses):
            p.resetJointState(self.robot_id, j, q)

        # ---------- End-effector target ----------
        self.target_pos = np.array([0.5, 0.0, 0.4])
        self.target_orn = p.getQuaternionFromEuler([np.pi, 0, 0])  # GRIPPER DOWN

        # ---------- Target visualization ----------
        self.target_viz = p.createVisualShape(p.GEOM_SPHERE, radius=0.015)
        self.target_body = p.createMultiBody(
            baseVisualShapeIndex=self.target_viz,
            basePosition=self.target_pos.tolist()
        )

        self.gripper_open = 1.0

        # Disable default motors
        for j in range(p.getNumJoints(self.robot_id)):
            p.setJointMotorControl2(self.robot_id, j, p.VELOCITY_CONTROL, force=0)

    # ---------- Gripper ----------
    def set_gripper(self, open_amount):
        self.gripper_open = np.clip(open_amount, 0.0, 1.0)
        finger_pos = 0.04 * self.gripper_open
        for j in self.finger_joints:
            p.setJointMotorControl2(
                self.robot_id, j,
                p.POSITION_CONTROL,
                targetPosition=finger_pos,
                force=60
            )

    # ---------- Move target ----------
    def move_target(self, dx=0, dy=0, dz=0):
        self.target_pos += np.array([dx, dy, dz])

        self.target_pos[0] = np.clip(self.target_pos[0], 0.3, 0.75)
        self.target_pos[1] = np.clip(self.target_pos[1], -0.4, 0.4)
        self.target_pos[2] = np.clip(self.target_pos[2], 0.02, 0.8)

    # ---------- Simulation step ----------
    def step(self):
        joint_poses = p.calculateInverseKinematics(
            self.robot_id,
            self.ee_link_index,
            self.target_pos,
            self.target_orn,
            self.lower_limits,
            self.upper_limits,
            self.joint_ranges,
            self.rest_poses
        )

        for i, j in enumerate(self.arm_joints):
            p.setJointMotorControl2(
                self.robot_id,
                j,
                p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                force=200
            )

        self.set_gripper(self.gripper_open)

        # Update target marker
        p.resetBasePositionAndOrientation(
            self.target_body,
            self.target_pos.tolist(),
            [0, 0, 0, 1]
        )

        p.stepSimulation()
        time.sleep(1.0 / 240.0)
