from __future__ import annotations

import numpy as np

from simulation.soft_cylinder import SoftCylinder
from simulation.rectangular_tray import RectangularTray
from simulation.pick_and_place_controller import PickAndPlaceController


def test_cylinder_dimensions() -> None:
    assert abs(SoftCylinder.diameter - 0.02) < 1e-9
    assert abs(SoftCylinder.height - 0.18) < 1e-9


def test_tray_dimensions() -> None:
    assert abs(RectangularTray.length - 0.30) < 1e-9
    assert abs(RectangularTray.width - 0.15) < 1e-9
    assert abs(RectangularTray.height - 0.005) < 1e-9


def test_workspace_validation_accepts_and_rejects() -> None:
    assert PickAndPlaceController._is_valid_workspace_target(np.array([0.5, 0.05, 0.4]))
    assert PickAndPlaceController._is_valid_workspace_target(np.array([0.30, -0.10, 0.4]))
    assert not PickAndPlaceController._is_valid_workspace_target(np.array([0.10, 0.05, 0.4]))
    assert not PickAndPlaceController._is_valid_workspace_target(np.array([0.5, 0.80, 0.4]))


def test_placement_bounds_logic() -> None:
    tray = np.array([0.5, 0.28, 0.41])
    margin = 0.008
    inside = np.array([0.52, 0.29, 0.42])
    outside = np.array([0.70, 0.28, 0.42])
    assert abs(inside[0] - tray[0]) <= RectangularTray.length / 2 - margin
    assert abs(inside[1] - tray[1]) <= RectangularTray.width / 2 - margin
    assert not (abs(outside[0] - tray[0]) <= RectangularTray.length / 2 - margin)


def test_scene_loads() -> None:
    from pathlib import Path
    import sys

    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    from scripts.generate_soft_cylinder import build_xml, OUTPUT

    if not OUTPUT.exists():
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT.write_text(build_xml())
    from simulation.environment import SimulationEnvironment

    env = SimulationEnvironment()
    assert env.model.nbody > 0
    assert env.model.nq > 0
