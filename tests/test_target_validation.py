from __future__ import annotations

import numpy as np

from simulation.pick_and_place_controller import PickAndPlaceController


def test_valid_workspace_targets() -> None:
    assert PickAndPlaceController._is_valid_workspace_target(np.array([0.45, 0.10]))
    assert PickAndPlaceController._is_valid_workspace_target(np.array([0.60, 0.30]))


def test_invalid_workspace_targets() -> None:
    assert not PickAndPlaceController._is_valid_workspace_target(np.array([0.0, 0.0]))
    assert not PickAndPlaceController._is_valid_workspace_target(np.array([1.0, 0.2]))
    assert not PickAndPlaceController._is_valid_workspace_target(np.array([0.5, 0.50]))
