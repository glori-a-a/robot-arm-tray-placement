from __future__ import annotations

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

ARM_JOINT_COUNT = 7
GRIPPER_OPEN = 255.0
GRIPPER_CLOSED = 20.0


class RobotArm:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        self.hand_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
        self.gripper_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripper_site")
        self.dof_ids = np.arange(ARM_JOINT_COUNT)
        self.gripper_actuator_id = ARM_JOINT_COUNT
        self.post_step = None
        self.position_tolerance = 0.008
        self.rotation_tolerance = 0.08

    @property
    def gripper_position(self) -> np.ndarray:
        return self.data.site_xpos[self.gripper_site_id].copy()

    @property
    def gripper_orientation(self) -> np.ndarray:
        return self.data.site_xmat[self.gripper_site_id].reshape(3, 3).copy()

    @property
    def gripper_command(self) -> float:
        return float(self.data.ctrl[self.gripper_actuator_id])

    def set_gripper(self, opening: float) -> None:
        self.data.ctrl[self.gripper_actuator_id] = opening

    def open_gripper(self) -> None:
        self.set_gripper(GRIPPER_OPEN)

    def close_gripper(self) -> None:
        self.set_gripper(GRIPPER_CLOSED)

    def hold_current_pose(self) -> None:
        self.data.ctrl[:ARM_JOINT_COUNT] = self.data.qpos[:ARM_JOINT_COUNT]

    def move_gripper_to(
        self,
        target_position: np.ndarray,
        target_rotation: np.ndarray | None = None,
        gripper_opening: float | None = None,
        step_count: int = 400,
        position_gain: float = 2.5,
        rotation_gain: float = 1.0,
        damping: float = 0.08,
        on_step=None,
        position_tolerance: float | None = None,
        rotation_tolerance: float | None = None,
    ) -> bool:
        if target_rotation is None:
            # grasp pose
            target_rotation = np.array(
                [
                    [0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0],
                ]
            )

        pos_tol = self.position_tolerance if position_tolerance is None else position_tolerance
        rot_tol = self.rotation_tolerance if rotation_tolerance is None else rotation_tolerance

        jac_pos = np.zeros((3, self.model.nv))
        jac_rot = np.zeros((3, self.model.nv))
        error_pos = np.zeros(3)
        error_rot = np.zeros(3)
        reached = False

        for _ in range(step_count):
            current_position = self.gripper_position
            current_rotation = self.gripper_orientation

            error_pos[:] = target_position - current_position
            target_rot = Rotation.from_matrix(target_rotation)
            current_rot = Rotation.from_matrix(current_rotation)
            error_rot[:] = (target_rot * current_rot.inv()).as_rotvec()

            if np.linalg.norm(error_pos) < pos_tol and np.linalg.norm(error_rot) < rot_tol:
                reached = True
                break

            mujoco.mj_jacSite(self.model, self.data, jac_pos, jac_rot, self.gripper_site_id)
            jacobian = np.vstack([jac_pos[:, self.dof_ids], jac_rot[:, self.dof_ids]])
            error = np.concatenate([position_gain * error_pos, rotation_gain * error_rot])
            joint_delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(6), error
            )

            self.data.ctrl[:ARM_JOINT_COUNT] = self.data.qpos[:ARM_JOINT_COUNT] + joint_delta
            if gripper_opening is not None:
                self.set_gripper(gripper_opening)
            mujoco.mj_step(self.model, self.data)
            if self.post_step is not None:
                self.post_step()
            if on_step is not None:
                on_step()

        if not reached:
            error_pos[:] = target_position - self.gripper_position
            target_rot = Rotation.from_matrix(target_rotation)
            current_rot = Rotation.from_matrix(self.gripper_orientation)
            error_rot[:] = (target_rot * current_rot.inv()).as_rotvec()
            reached = np.linalg.norm(error_pos) < pos_tol * 1.5 and np.linalg.norm(error_rot) < rot_tol * 1.5

        return bool(reached)

    def wait(self, step_count: int = 200, on_step=None) -> None:
        self.hold_current_pose()
        for _ in range(step_count):
            mujoco.mj_step(self.model, self.data)
            if self.post_step is not None:
                self.post_step()
            if on_step is not None:
                on_step()
