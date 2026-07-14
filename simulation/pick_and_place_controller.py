from __future__ import annotations

from enum import Enum, auto

import numpy as np
import mujoco

from simulation.robot_arm import RobotArm, GRIPPER_OPEN, GRIPPER_CLOSED
from simulation.soft_cylinder import SoftCylinder
from simulation.rectangular_tray import RectangularTray
from sensors.object_detector import ObjectDetector


class TaskState(Enum):
    INITIALISING = auto()
    DETECTING = auto()
    APPROACHING = auto()
    GRASPING = auto()
    LIFTING = auto()
    TRANSPORTING = auto()
    PLACING = auto()
    VERIFYING = auto()
    COMPLETE = auto()
    FAILED = auto()


class PickAndPlaceController:
    TABLE_TOP_Z = 0.40
    APPROACH_OFFSET = 0.16
    GRASP_LIFT = 0.12
    FREE_JOINT_START = 9
    RELEASE_CLEARANCE = 0.018
    GRASP_XY_TOL = 0.035
    GRASP_Z_TOL = 0.050
    PLACEMENT_MARGIN = 0.008
    LINEAR_SPEED_TOL = 0.05
    ANGULAR_SPEED_TOL = 1.0
    SETTLE_STEPS = 600

    def __init__(
        self,
        environment,
        use_sensor: bool = True,
        placement_mode: str = "physical_release",
        cylinder_xy: np.ndarray | None = None,
        tray_xy: np.ndarray | None = None,
    ):
        self.environment = environment
        self.model = environment.model
        self.data = environment.data
        self.robot = RobotArm(self.model, self.data)
        self.detector = ObjectDetector(self.model, self.data) if use_sensor else None
        self.use_sensor = use_sensor
        self.placement_mode = placement_mode
        self.state = TaskState.INITIALISING
        self.cylinder_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, SoftCylinder.body_name
        )
        self.tray_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, RectangularTray.body_name
        )
        self.cylinder_offset: np.ndarray | None = None
        self.cylinder_grasp_quat: np.ndarray | None = None
        self._pinned_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._min_cylinder_z = self._cylinder_center_on_table()
        self.robot.post_step = self._on_post_step

        self._configured_cylinder_xy = (
            np.asarray(cylinder_xy, dtype=float)
            if cylinder_xy is not None
            else np.array([0.5, 0.05])
        )
        self._configured_tray_xy = (
            np.asarray(tray_xy, dtype=float)
            if tray_xy is not None
            else np.array([0.5, 0.28])
        )
        self._apply_scene_layout()
        self._set_state(TaskState.INITIALISING)

    def _set_state(self, state: TaskState) -> None:
        self.state = state
        print(f"[state] {state.name}")

    def _failure_result(self, reason: str, **extra) -> dict:
        self._set_state(TaskState.FAILED)
        result = {
            "success": False,
            "reason": reason,
            "state": self.state.name,
            "cylinder_position": self._cylinder_position(),
            "tray_center": self.environment.site_position(RectangularTray.center_site_name),
        }
        result.update(extra)
        print(f"[result] FAILED reason={reason}")
        return result

    def _cylinder_center_on_table(self) -> float:
        return SoftCylinder.center_z_on_surface(self.TABLE_TOP_Z)

    def _cylinder_center_on_tray(self) -> float:
        return SoftCylinder.center_z_on_surface(RectangularTray.top_surface_z(self.TABLE_TOP_Z))

    def _default_cylinder_position(self) -> np.ndarray:
        return np.array(
            [
                float(self._configured_cylinder_xy[0]),
                float(self._configured_cylinder_xy[1]),
                self._cylinder_center_on_table(),
            ]
        )

    def _default_tray_position(self) -> np.ndarray:
        return np.array(
            [
                float(self._configured_tray_xy[0]),
                float(self._configured_tray_xy[1]),
                RectangularTray.placement_height(self.TABLE_TOP_Z),
            ]
        )

    def _apply_scene_layout(self) -> None:
        tray_pos = self._default_tray_position()
        self.model.body_pos[self.tray_body_id, 0] = tray_pos[0]
        self.model.body_pos[self.tray_body_id, 1] = tray_pos[1]
        self.model.body_pos[self.tray_body_id, 2] = tray_pos[2]
        self._pin_at(self._default_cylinder_position(), self._flat_cylinder_quat())

    @staticmethod
    def _is_valid_workspace_target(position: np.ndarray) -> bool:
        return bool(0.30 <= position[0] <= 0.70 and -0.10 <= position[1] <= 0.45)

    def _resolve_targets(self) -> tuple[np.ndarray, np.ndarray]:
        self._set_state(TaskState.DETECTING)
        cylinder_target = self._default_cylinder_position()
        tray_target = self._default_tray_position()

        if self.detector is None:
            print("[sensor] disabled — using configured scene poses")
            return cylinder_target, tray_target

        detection = self.detector.detect(table_z=self.TABLE_TOP_Z)

        if detection["cylinder_position"] is not None:
            detected = detection["cylinder_position"]
            print(f"[sensor] Cylinder detected at x={detected[0]:.3f}, y={detected[1]:.3f}")
            if self._is_valid_workspace_target(detected):
                cylinder_target[0] = float(detected[0])
                cylinder_target[1] = float(detected[1])
            else:
                print("[sensor] Cylinder detection outside workspace — fallback to configured pose")
        else:
            print("[sensor] Cylinder detection failed — fallback to configured pose")

        if detection["tray_position"] is not None:
            detected = detection["tray_position"]
            print(f"[sensor] Tray detected at x={detected[0]:.3f}, y={detected[1]:.3f}")
            if self._is_valid_workspace_target(detected):
                tray_target[0] = float(detected[0])
                tray_target[1] = float(detected[1])
            else:
                print("[sensor] Tray detection outside workspace — fallback to configured pose")
        else:
            print("[sensor] Tray detection failed — fallback to configured pose")

        return cylinder_target, tray_target

    def _cylinder_qpos_start(self) -> int:
        return self.FREE_JOINT_START

    def _cylinder_position(self) -> np.ndarray:
        return self.data.xpos[self.cylinder_body_id].copy()

    def _cylinder_speeds(self) -> tuple[float, float]:
        start = self._cylinder_qpos_start()
        linear = float(np.linalg.norm(self.data.qvel[start : start + 3]))
        angular = float(np.linalg.norm(self.data.qvel[start + 3 : start + 6]))
        return linear, angular

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

    def _release_constraints(self) -> None:
        self.cylinder_offset = None
        self._pinned_pose = None

    def _lower_pinned_cylinder_to(
        self,
        end_pos: np.ndarray,
        quat: np.ndarray,
        on_step=None,
        steps: int = 40,
    ) -> None:
        start_pos = self._cylinder_position().copy()
        start_pos[0] = end_pos[0]
        start_pos[1] = end_pos[1]
        end_z = float(end_pos[2])
        if start_pos[2] < end_z:
            start_pos[2] = end_z

        self.cylinder_offset = None
        self._pin_at(start_pos, quat)

        gripper_xy = self.robot.gripper_position.copy()
        for i in range(1, steps + 1):
            alpha = i / steps
            t = alpha * alpha * (3.0 - 2.0 * alpha)
            pos = start_pos * (1.0 - t) + end_pos * t
            self._pin_at(pos, quat)

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

    def _validate_grasp_alignment(self) -> tuple[bool, dict]:
        grip = self.robot.gripper_position
        cyl = self._cylinder_position()
        xy_error = float(np.linalg.norm(grip[:2] - cyl[:2]))
        z_error = float(abs(grip[2] - cyl[2]))
        gripper_closed = bool(self.robot.gripper_command <= (GRIPPER_CLOSED + 30.0))
        diagnostics = {
            "xy_error": xy_error,
            "z_error": z_error,
            "gripper_closed": gripper_closed,
            "gripper_command": self.robot.gripper_command,
        }
        valid = (
            xy_error < self.GRASP_XY_TOL
            and z_error < self.GRASP_Z_TOL
            and gripper_closed
        )
        return valid, diagnostics

    def _move_and_carry(self, target_position: np.ndarray, on_step=None, **kwargs) -> bool:
        step_count = kwargs.pop("step_count", 400)
        gripper_opening = kwargs.pop("gripper_opening", None)

        for _ in range(step_count):
            current = self.robot.gripper_position
            error = target_position - current
            if np.linalg.norm(error) < 0.008:
                return True

            step_target = current + np.clip(error, -0.01, 0.01) * 2.5
            self.robot.move_gripper_to(
                step_target,
                gripper_opening=gripper_opening,
                step_count=1,
                on_step=on_step,
                position_tolerance=0.01,
            )
        return bool(np.linalg.norm(target_position - self.robot.gripper_position) < 0.025)

    def _evaluate_placement(self) -> dict:
        cylinder = self._cylinder_position()
        tray = self.environment.site_position(RectangularTray.center_site_name)
        tray_rest_z = self._cylinder_center_on_tray()
        linear_speed, angular_speed = self._cylinder_speeds()

        inside_x = abs(cylinder[0] - tray[0]) <= RectangularTray.length / 2 - self.PLACEMENT_MARGIN
        inside_y = abs(cylinder[1] - tray[1]) <= RectangularTray.width / 2 - self.PLACEMENT_MARGIN
        height_ok = cylinder[2] >= tray_rest_z - 0.01 and cylinder[2] <= tray_rest_z + 0.05
        stable = linear_speed < self.LINEAR_SPEED_TOL and angular_speed < self.ANGULAR_SPEED_TOL
        success = bool(inside_x and inside_y and height_ok and stable)

        reason = None
        if not inside_x:
            reason = "outside_tray_x"
        elif not inside_y:
            reason = "outside_tray_y"
        elif not height_ok:
            reason = "height_invalid"
        elif not stable:
            reason = "unstable"

        print(f"[evaluation] inside_x={inside_x}")
        print(f"[evaluation] inside_y={inside_y}")
        print(f"[evaluation] height_ok={height_ok}")
        print(f"[evaluation] stable={stable}")

        return {
            "cylinder_position": cylinder,
            "tray_center": tray,
            "inside_x": inside_x,
            "inside_y": inside_y,
            "height_ok": height_ok,
            "linear_speed": linear_speed,
            "angular_speed": angular_speed,
            "stable": stable,
            "success": success,
            "reason": reason,
            "distance": float(np.linalg.norm(cylinder[:2] - tray[:2])),
        }

    def _place_physical_release(self, tray_target: np.ndarray, tray_rest_z: float, on_step=None) -> dict | None:
        release_pos = self._cylinder_position().copy()
        release_pos[0] = tray_target[0]
        release_pos[1] = tray_target[1]
        release_pos[2] = tray_rest_z + self.RELEASE_CLEARANCE

        if not self._move_and_carry(
            self._gripper_for_cylinder(release_pos),
            gripper_opening=GRIPPER_CLOSED,
            step_count=350,
            on_step=on_step,
        ):
            return self._failure_result("release_lower_ik_failed")

        print(f"[place] physical_release at z={self._cylinder_position()[2]:.4f}")
        self._release_constraints()
        self.robot.hold_current_pose()
        self.robot.open_gripper()
        self._settle_released_object(on_step=on_step)
        return None

    def _settle_released_object(self, on_step=None) -> None:
        start = self._cylinder_qpos_start()
        for _ in range(self.SETTLE_STEPS):
            self.robot.hold_current_pose()
            mujoco.mj_step(self.model, self.data)
            # Mild free-joint damping to emulate energy loss in compliant contact.
            self.data.qvel[start : start + 6] *= 0.985
            if on_step is not None:
                on_step()

    def _place_assisted_release(self, tray_target: np.ndarray, tray_rest_z: float, on_step=None) -> dict | None:
        hover_pos = self._cylinder_position().copy()
        hover_pos[0] = tray_target[0]
        hover_pos[1] = tray_target[1]
        if hover_pos[2] < tray_rest_z + 0.04:
            hover_pos[2] = tray_rest_z + 0.04

        start = self._cylinder_qpos_start()
        grasp_quat = self.data.qpos[start + 3 : start + 7].copy()
        self._pin_at(hover_pos, grasp_quat)
        print(f"[place] assisted_release pin z={hover_pos[2]:.4f}")

        tray_cylinder = tray_target.copy()
        tray_cylinder[2] = tray_rest_z
        self._lower_pinned_cylinder_to(tray_cylinder, grasp_quat, on_step=on_step, steps=40)
        self._release_constraints()
        self.robot.hold_current_pose()
        self.robot.open_gripper()
        self.robot.wait(150, on_step=on_step)
        return None

    def run(self, on_step=None) -> dict:
        print("Starting pick-and-place task...")
        self.robot.open_gripper()
        self.robot.wait(150, on_step=on_step)

        cylinder_target, tray_target = self._resolve_targets()

        # approach
        self._set_state(TaskState.APPROACHING)
        above_cylinder = cylinder_target.copy()
        above_cylinder[2] += self.APPROACH_OFFSET
        if not self.robot.move_gripper_to(
            above_cylinder, gripper_opening=GRIPPER_OPEN, step_count=500, on_step=on_step
        ):
            return self._failure_result("approach_ik_failed")

        # grasp
        self._set_state(TaskState.GRASPING)
        grasp_point = cylinder_target.copy()
        grasp_point[2] = self._cylinder_center_on_table() + 0.012
        if not self.robot.move_gripper_to(
            grasp_point, gripper_opening=GRIPPER_OPEN, step_count=350, on_step=on_step
        ):
            return self._failure_result("grasp_ik_failed")

        self.robot.close_gripper()
        self.robot.wait(250, on_step=on_step)

        valid, diagnostics = self._validate_grasp_alignment()
        print(
            f"[grasp] valid={valid} xy_error={diagnostics['xy_error']:.4f} "
            f"z_error={diagnostics['z_error']:.4f} closed={diagnostics['gripper_closed']}"
        )
        if not valid:
            return self._failure_result("grasp_validation_failed", diagnostics=diagnostics)

        self._begin_cylinder_attach()
        print("Cylinder constraint activated after grasp validation.")

        # pick
        self._set_state(TaskState.LIFTING)
        lift_cylinder = cylinder_target.copy()
        lift_cylinder[2] += self.GRASP_LIFT
        if not self._move_and_carry(
            self._gripper_for_cylinder(lift_cylinder),
            gripper_opening=GRIPPER_CLOSED,
            step_count=450,
            on_step=on_step,
        ):
            return self._failure_result("lift_ik_failed")

        # move
        self._set_state(TaskState.TRANSPORTING)
        tray_rest_z = self._cylinder_center_on_tray()
        above_tray_cylinder = tray_target.copy()
        above_tray_cylinder[2] = tray_rest_z + 0.10
        self._min_cylinder_z = tray_rest_z
        if not self._move_and_carry(
            self._gripper_for_cylinder(above_tray_cylinder),
            gripper_opening=GRIPPER_CLOSED,
            step_count=500,
            on_step=on_step,
        ):
            return self._failure_result("transport_ik_failed")
        self.robot.wait(80, on_step=on_step)

        # place
        self._set_state(TaskState.PLACING)
        if self.placement_mode == "assisted_release":
            failure = self._place_assisted_release(tray_target, tray_rest_z, on_step=on_step)
        else:
            failure = self._place_physical_release(tray_target, tray_rest_z, on_step=on_step)
        if failure is not None:
            return failure

        retreat = self.robot.gripper_position.copy()
        retreat[2] += 0.14
        if not self.robot.move_gripper_to(
            retreat, gripper_opening=GRIPPER_OPEN, step_count=300, on_step=on_step
        ):
            print("[warn] retreat_ik_failed — continuing to placement evaluation")
        self.robot.wait(120, on_step=on_step)

        self._set_state(TaskState.VERIFYING)
        evaluation = self._evaluate_placement()
        if evaluation["success"]:
            self._set_state(TaskState.COMPLETE)
            print("[result] SUCCESS")
        else:
            self._set_state(TaskState.FAILED)
            print(f"[result] FAILED reason={evaluation['reason']}")

        return {
            "success": evaluation["success"],
            "reason": evaluation["reason"],
            "state": self.state.name,
            "cylinder_position": evaluation["cylinder_position"],
            "tray_center": evaluation["tray_center"],
            "distance": evaluation["distance"],
            "evaluation": evaluation,
            "placement_mode": self.placement_mode,
        }

    def close(self) -> None:
        if self.detector is not None:
            self.detector.close()
