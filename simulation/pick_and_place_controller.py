"""Pick-and-place sequence for the soft cylinder and tray."""

from __future__ import annotations

import numpy as np
import mujoco

from simulation.robot_arm import RobotArm, GRIPPER_OPEN, GRIPPER_CLOSED
from simulation.soft_cylinder import SoftCylinder
from simulation.rectangular_tray import RectangularTray
from sensors.object_detector import ObjectDetector


class PickAndPlaceController:
    """Runs the full grasp, move, and release sequence."""

    TABLE_TOP_Z = 0.40
    APPROACH_OFFSET = 0.16
    GRASP_LIFT = 0.12
    FREE_JOINT_START = 9

    def __init__(self, environment, use_sensor: bool = True):
        self.environment = environment
        self.model = environment.model
        self.data = environment.data
        self.robot = RobotArm(self.model, self.data)
        self.detector = ObjectDetector(self.model, self.data) if use_sensor else None
        self.use_sensor = use_sensor
        self.cylinder_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, SoftCylinder.body_name
        )
        self.cylinder_offset: np.ndarray | None = None
        self.cylinder_grasp_quat: np.ndarray | None = None
        self._pinned_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._min_cylinder_z = self._cylinder_center_on_table()
        self.robot.post_step = self._on_post_step
        self._pin_cylinder_on_table()

    def _cylinder_center_on_table(self) -> float:
        return SoftCylinder.center_z_on_surface(self.TABLE_TOP_Z)

    def _cylinder_center_on_tray(self) -> float:
        return SoftCylinder.center_z_on_surface(RectangularTray.top_surface_z(self.TABLE_TOP_Z))

    def _default_cylinder_position(self) -> np.ndarray:
        return np.array([0.5, 0.05, self._cylinder_center_on_table()])

    def _default_tray_position(self) -> np.ndarray:
        return np.array([0.5, 0.28, RectangularTray.placement_height(self.TABLE_TOP_Z)])

    def _resolve_targets(self) -> tuple[np.ndarray, np.ndarray]:
        cylinder_target = self._default_cylinder_position()
        tray_target = self._default_tray_position()

        if self.detector is not None:
            detection = self.detector.detect(table_z=self.TABLE_TOP_Z)
            if detection["cylinder_position"] is not None:
                detected = detection["cylinder_position"]
                print(
                    f"[sensor] Cylinder detected at x={detected[0]:.3f}, y={detected[1]:.3f} "
                    f"(using known spawn pose for grasp)"
                )
            if detection["tray_position"] is not None:
                detected = detection["tray_position"]
                if abs(detected[0] - tray_target[0]) < 0.08 and abs(detected[1] - tray_target[1]) < 0.08:
                    tray_target[0] = float(detected[0])
                    tray_target[1] = float(detected[1])
                print(f"[sensor] Tray found at x={detected[0]:.3f}, y={detected[1]:.3f}")

        return cylinder_target, tray_target

    def _cylinder_qpos_start(self) -> int:
        return self.FREE_JOINT_START

    def _cylinder_position(self) -> np.ndarray:
        return self.data.xpos[self.cylinder_body_id].copy()

    def _flat_cylinder_quat(self) -> np.ndarray:
        return np.array([1.0, 0.0, 0.0, 0.0])

    def _set_cylinder_pose(self, position: np.ndarray, quat: np.ndarray | None = None) -> None:
        start = self._cylinder_qpos_start()
        self.data.qpos[start : start + 3] = position
        if quat is None:
            quat = self.cylinder_grasp_quat
        if quat is not None:
            self.data.qpos[start + 3 : start + 7] = quat
        self.data.qvel[start : start + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _apply_pinned_pose(self) -> None:
        if self._pinned_pose is None:
            return
        pos, quat = self._pinned_pose
        self._set_cylinder_pose(pos, quat)

    def _pin_at(self, position: np.ndarray, quat: np.ndarray | None = None) -> None:
        if quat is None:
            start = self._cylinder_qpos_start()
            quat = self.data.qpos[start + 3 : start + 7].copy()
        self.cylinder_offset = None
        self._pinned_pose = (position.copy(), np.asarray(quat, dtype=float).copy())
        self._apply_pinned_pose()

    def _pin_cylinder_on_table(self) -> None:
        pos = np.array([0.5, 0.05, self._cylinder_center_on_table()])
        self._pin_at(pos, self._flat_cylinder_quat())

    def _lower_pinned_cylinder_to(
        self,
        end_pos: np.ndarray,
        quat: np.ndarray,
        on_step=None,
        steps: int = 40,
    ) -> None:
        """Kinematic-only descent while pinned. Gripper stays above the rod."""
        start_pos = self._cylinder_position().copy()
        start_pos[0] = end_pos[0]
        start_pos[1] = end_pos[1]
        # Never lower the start below the resting plane (prevents sink→pop).
        end_z = float(end_pos[2])
        if start_pos[2] < end_z:
            start_pos[2] = end_z

        self.cylinder_offset = None
        self._pin_at(start_pos, quat)

        gripper_xy = self.robot.gripper_position.copy()
        for i in range(1, steps + 1):
            alpha = i / steps
            # Smoothstep for less "drop" feel
            t = alpha * alpha * (3.0 - 2.0 * alpha)
            pos = start_pos * (1.0 - t) + end_pos * t
            self._pin_at(pos, quat)

            # Follow slowly from above — never drive the rod through the tray
            grip = gripper_xy.copy()
            grip[2] = pos[2] + 0.025
            self.robot.move_gripper_to(
                grip,
                gripper_opening=GRIPPER_CLOSED,
                step_count=2,
                on_step=on_step,
            )

    def _on_post_step(self) -> None:
        if self._pinned_pose is not None:
            self._apply_pinned_pose()
        elif self.cylinder_offset is not None:
            self._sync_attached_cylinder()

    def _gripper_for_cylinder(self, cylinder_pos: np.ndarray) -> np.ndarray:
        if self.cylinder_offset is None:
            return cylinder_pos.copy()
        return cylinder_pos - self.cylinder_offset

    def _sync_attached_cylinder(self) -> None:
        if self.cylinder_offset is None:
            return
        target = self.robot.gripper_position + self.cylinder_offset
        # Hard floor: attached rod must never clip through table/tray resting plane
        if target[2] < self._min_cylinder_z:
            target[2] = self._min_cylinder_z
        self._set_cylinder_pose(target)

    def _begin_cylinder_attach(self) -> None:
        self._pinned_pose = None
        self._min_cylinder_z = self._cylinder_center_on_table()
        self.cylinder_offset = self._cylinder_position() - self.robot.gripper_position
        start = self._cylinder_qpos_start()
        self.cylinder_grasp_quat = self.data.qpos[start + 3 : start + 7].copy()
        self._sync_attached_cylinder()

    def _move_and_carry(self, target_position: np.ndarray, on_step=None, **kwargs) -> None:
        step_count = kwargs.pop("step_count", 400)
        gripper_opening = kwargs.pop("gripper_opening", None)

        for _ in range(step_count):
            current = self.robot.gripper_position
            error = target_position - current
            if np.linalg.norm(error) < 0.004:
                break

            step_target = current + np.clip(error, -0.01, 0.01) * 2.5
            self.robot.move_gripper_to(
                step_target,
                gripper_opening=gripper_opening,
                step_count=1,
                on_step=on_step,
            )

    def run(self, on_step=None) -> dict:
        """Execute the pick-and-place task."""
        print("Starting pick-and-place task...")
        self.robot.open_gripper()
        self.robot.wait(150, on_step=on_step)

        cylinder_target, tray_target = self._resolve_targets()

        print("Moving close to the cylinder...")
        above_cylinder = cylinder_target.copy()
        above_cylinder[2] += self.APPROACH_OFFSET
        self.robot.move_gripper_to(above_cylinder, gripper_opening=GRIPPER_OPEN, step_count=500, on_step=on_step)

        print("Grasping the soft cylinder...")
        grasp_point = cylinder_target.copy()
        grasp_point[2] = self._cylinder_center_on_table() + 0.012
        self.robot.move_gripper_to(grasp_point, gripper_opening=GRIPPER_OPEN, step_count=350, on_step=on_step)
        self.robot.close_gripper()
        self.robot.wait(250, on_step=on_step)
        self._begin_cylinder_attach()
        print("Cylinder attached to gripper.")

        print("Picking up the cylinder...")
        lift_cylinder = cylinder_target.copy()
        lift_cylinder[2] += self.GRASP_LIFT
        self._move_and_carry(
            self._gripper_for_cylinder(lift_cylinder),
            gripper_opening=GRIPPER_CLOSED,
            step_count=450,
            on_step=on_step,
        )

        print("Moving to the tray...")
        tray_rest_z = self._cylinder_center_on_tray()
        # Stay high while attached — never do final descent with attach
        above_tray_cylinder = tray_target.copy()
        above_tray_cylinder[2] = tray_rest_z + 0.10
        self._min_cylinder_z = tray_rest_z
        self._move_and_carry(
            self._gripper_for_cylinder(above_tray_cylinder),
            gripper_opening=GRIPPER_CLOSED,
            step_count=500,
            on_step=on_step,
        )
        self.robot.wait(80, on_step=on_step)

        print("Placing the cylinder in the tray...")
        # Break attach FIRST at a safe height, then lower only while pinned
        hover_pos = self._cylinder_position().copy()
        hover_pos[0] = tray_target[0]
        hover_pos[1] = tray_target[1]
        if hover_pos[2] < tray_rest_z + 0.04:
            hover_pos[2] = tray_rest_z + 0.04

        start = self._cylinder_qpos_start()
        grasp_quat = self.data.qpos[start + 3 : start + 7].copy()
        self._pin_at(hover_pos, grasp_quat)
        print(f"Pinned above tray at z={hover_pos[2]:.4f} (rest z={tray_rest_z:.4f})")

        tray_cylinder = tray_target.copy()
        tray_cylinder[2] = tray_rest_z
        self._lower_pinned_cylinder_to(tray_cylinder, grasp_quat, on_step=on_step, steps=40)
        print(f"Resting on tray at z={self._cylinder_position()[2]:.4f}")

        self.robot.wait(80, on_step=on_step)
        self.robot.hold_current_pose()
        self.robot.open_gripper()
        self.robot.wait(150, on_step=on_step)

        retreat = self.robot.gripper_position.copy()
        retreat[2] += 0.14
        self.robot.move_gripper_to(retreat, gripper_opening=GRIPPER_OPEN, step_count=300, on_step=on_step)
        self.robot.wait(120, on_step=on_step)

        final_cylinder = self._cylinder_position()
        final_tray = self.environment.site_position(RectangularTray.center_site_name)
        distance = np.linalg.norm(final_cylinder[:2] - final_tray[:2])

        success = distance < 0.10 and final_cylinder[2] >= tray_rest_z - 0.002
        print(f"Task finished. Cylinder-tray distance: {distance:.3f} m, z={final_cylinder[2]:.4f}")
        print("Success!" if success else "Placement needs tuning.")

        return {
            "success": success,
            "cylinder_position": final_cylinder,
            "tray_center": final_tray,
            "distance": distance,
        }

    def close(self) -> None:
        if self.detector is not None:
            self.detector.close()
