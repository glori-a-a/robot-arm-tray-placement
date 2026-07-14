from __future__ import annotations

import numpy as np

from simulation.environment import SimulationEnvironment
from simulation.pick_and_place_controller import PickAndPlaceController
from simulation.rectangular_tray import RectangularTray
from scripts.generate_soft_cylinder import build_xml, OUTPUT


def _controller() -> PickAndPlaceController:
    if not OUTPUT.exists():
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT.write_text(build_xml())
    env = SimulationEnvironment()
    return PickAndPlaceController(env, use_sensor=False)


def test_evaluate_placement_inside() -> None:
    controller = _controller()
    tray = controller.environment.site_position(RectangularTray.center_site_name)
    rest_z = controller._cylinder_center_on_tray()
    controller._set_cylinder_pose(np.array([tray[0], tray[1], rest_z]), controller._flat_cylinder_quat())
    controller._release_constraints()
    result = controller._evaluate_placement()
    controller.close()
    assert result["inside_x"]
    assert result["inside_y"]
    assert result["height_ok"]


def test_evaluate_placement_outside() -> None:
    controller = _controller()
    rest_z = controller._cylinder_center_on_tray()
    controller._set_cylinder_pose(np.array([0.70, 0.28, rest_z]), controller._flat_cylinder_quat())
    controller._release_constraints()
    result = controller._evaluate_placement()
    controller.close()
    assert not result["success"]
    assert not result["inside_x"]
