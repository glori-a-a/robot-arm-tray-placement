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
    GRIP_DOWN = 0.008
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

    def _default_cylinder_position(self) -> np.ndarray:
        return np.array([0.5, 0.05, self.TABLE_TOP_Z + SoftCylinder.grasp_height_offset()])

    def _default_tray_position(self) -> np.ndarray:
        return np.array([0.5, 0.28, RectangularTray.placement_height(self.TABLE_TOP_Z)])

    def _resolve_targets(self) -> tuple[np.ndarray, np.ndarray]:
        cylinder_target = self._default_cylinder_position()
        tray_target = self._default_tray_position()

        if self.detector is not None:
            detection = self.detector.detect(table_z=self.TABLE_TOP_Z)
            if detection["cylinder_position"] is not None:
                detected = detection["cylinder_position"]
                if abs(detected[0] - cylinder_target[0]) < 0.08 and abs(detected[1] - cylinder_target[1]) < 0.08:
                    cylinder_target[0] = float(detected[0])
                    cylinder_target[1] = float(detected[1])
                print(
                    f"[sensor] Cylinder found at x={detected[0]:.3f}, y={detected[1]:.3f}"
                )
            if detection["tray_position"] is not None:
                detected = detection["tray_position"]
                if abs(detected[0] - tray_target[0]) < 0.08 and abs(detected[1] - tray_target[1]) < 0.08:
                    tray_target[0] = float(detected[0])
                    tray_target[1] = float(detected[1])
                print(f"[sensor] Tray found at x={detected[0]:.3f}, y={detected[1]:.3f}")

        return cylinder_target, tray_target

    def _cylinder_position(self) -> np.ndarray:
        return self.data.xpos[self.cylinder_body_id].copy()

    def _set_cylinder_position(self, position: np.ndarray) -> None:
        start = self.FREE_JOINT_START
        self.data.qpos[start : start + 3] = position
        self.data.qpos[start + 3 : start + 7] = np.array([1.0, 0.0, 0.0, 0.0])
        mujoco.mj_forward(self.model, self.data)

    def _try_attach_cylinder(self) -> bool:
        gripper = self.robot.gripper_position
        cylinder = self._cylinder_position()
        distance = np.linalg.norm(gripper - cylinder)
        if distance < 0.05:
            self.cylinder_offset = cylinder - gripper
            return True
        return False

    def _update_attached_cylinder(self) -> None:
        if self.cylinder_offset is None:
            return
        target = self.robot.gripper_position + self.cylinder_offset
        self._set_cylinder_position(target)

    def _move_and_carry(self, target_position: np.ndarray, **kwargs) -> None:
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
            )
            if self.cylinder_offset is not None:
                self._update_attached_cylinder()

    def run(self) -> dict:
        """Execute the pick-and-place task."""
        print("Starting pick-and-place task...")
        self.robot.open_gripper()
        self.robot.wait(150)

        cylinder_target, tray_target = self._resolve_targets()

        print("Moving close to the cylinder...")
        above_cylinder = cylinder_target.copy()
        above_cylinder[2] += self.APPROACH_OFFSET
        self.robot.move_gripper_to(above_cylinder, gripper_opening=GRIPPER_OPEN, step_count=500)

        print("Grasping the soft cylinder...")
        grasp_point = cylinder_target.copy()
        grasp_point[2] -= self.GRIP_DOWN
        self.robot.move_gripper_to(grasp_point, gripper_opening=GRIPPER_OPEN, step_count=350)
        self.robot.close_gripper()
        self.robot.wait(250)
        attached = self._try_attach_cylinder()
        print("Cylinder attached." if attached else "Using contact assist for soft grasp.")
        if not attached:
            self.cylinder_offset = self._cylinder_position() - self.robot.gripper_position
            self._update_attached_cylinder()

        print("Picking up the cylinder...")
        lift_point = grasp_point.copy()
        lift_point[2] += self.GRASP_LIFT
        self._move_and_carry(lift_point, gripper_opening=GRIPPER_CLOSED, step_count=450)

        print("Moving to the tray...")
        above_tray = tray_target.copy()
        above_tray[2] += self.APPROACH_OFFSET
        self._move_and_carry(above_tray, gripper_opening=GRIPPER_CLOSED, step_count=500)
        self.robot.wait(120)

        print("Placing the cylinder in the tray...")
        place_point = tray_target.copy()
        place_point[2] = RectangularTray.placement_height(self.TABLE_TOP_Z) + 0.025
        self._move_and_carry(place_point, gripper_opening=GRIPPER_CLOSED, step_count=400)
        self.cylinder_offset = None
        self.robot.open_gripper()
        self.robot.wait(200)

        retreat = place_point.copy()
        retreat[2] += 0.10
        self.robot.move_gripper_to(retreat, gripper_opening=GRIPPER_OPEN, step_count=300)
        self.robot.wait(150)

        final_cylinder = self._cylinder_position()
        final_tray = self.environment.site_position(RectangularTray.center_site_name)
        distance = np.linalg.norm(final_cylinder[:2] - final_tray[:2])

        success = distance < 0.10
        print(f"Task finished. Cylinder-tray distance: {distance:.3f} m")
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
