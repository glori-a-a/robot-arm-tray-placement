import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from scripts.generate_soft_cylinder import build_xml, OUTPUT as CYLINDER_XML
from simulation.environment import SimulationEnvironment
from simulation.pick_and_place_controller import PickAndPlaceController


def ensure_assets() -> None:
    if not CYLINDER_XML.exists():
        CYLINDER_XML.parent.mkdir(parents=True, exist_ok=True)
        CYLINDER_XML.write_text(build_xml())


def sample_layout(rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    for _ in range(50):
        cylinder_xy = np.array(
            [
                0.50 + rng.uniform(-0.03, 0.03),
                0.05 + rng.uniform(-0.02, 0.02),
            ]
        )
        tray_xy = np.array(
            [
                0.50 + rng.uniform(-0.03, 0.03),
                0.28 + rng.uniform(-0.02, 0.02),
            ]
        )
        if np.linalg.norm(cylinder_xy - tray_xy) < 0.16:
            continue
        if not (0.35 <= cylinder_xy[0] <= 0.65 and -0.02 <= cylinder_xy[1] <= 0.15):
            continue
        if not (0.35 <= tray_xy[0] <= 0.65 and 0.20 <= tray_xy[1] <= 0.38):
            continue
        return cylinder_xy, tray_xy
    return np.array([0.5, 0.05]), np.array([0.5, 0.28])


def main() -> None:
    parser = argparse.ArgumentParser(description="Robot arm pick-and-place simulation")
    parser.add_argument("--no-sensor", action="store_true", help="Skip camera detection")
    parser.add_argument("--viewer", action="store_true", help="Open interactive MuJoCo viewer")
    parser.add_argument("--randomize", action="store_true", help="Randomise cylinder/tray XY")
    parser.add_argument("--seed", type=int, default=None, help="RNG seed for --randomize")
    parser.add_argument(
        "--placement-mode",
        choices=("physical_release", "assisted_release"),
        default="physical_release",
        help="Placement strategy after transport",
    )
    args = parser.parse_args()

    ensure_assets()
    environment = SimulationEnvironment()

    cylinder_xy = None
    tray_xy = None
    if args.randomize:
        rng = np.random.default_rng(args.seed)
        cylinder_xy, tray_xy = sample_layout(rng)
        print(f"[randomize] seed={args.seed} cylinder_xy={cylinder_xy} tray_xy={tray_xy}")

    controller = PickAndPlaceController(
        environment,
        use_sensor=not args.no_sensor,
        placement_mode=args.placement_mode,
        cylinder_xy=cylinder_xy,
        tray_xy=tray_xy,
    )

    if args.viewer:
        import mujoco.viewer

        with mujoco.viewer.launch_passive(environment.model, environment.data) as viewer:
            result = controller.run()
            while viewer.is_running():
                viewer.sync()
    else:
        result = controller.run()

    controller.close()
    sys.exit(0 if result["success"] else 1)


if __name__ == "__main__":
    main()
