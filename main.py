"""Entry point for the robot arm pick-and-place demo."""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from scripts.generate_soft_cylinder import build_xml, OUTPUT as CYLINDER_XML
from simulation.environment import SimulationEnvironment
from simulation.pick_and_place_controller import PickAndPlaceController


def ensure_assets() -> None:
    if not CYLINDER_XML.exists():
        CYLINDER_XML.parent.mkdir(parents=True, exist_ok=True)
        CYLINDER_XML.write_text(build_xml())


def main() -> None:
    parser = argparse.ArgumentParser(description="Robot arm pick-and-place simulation")
    parser.add_argument("--no-sensor", action="store_true", help="Skip camera detection")
    parser.add_argument("--viewer", action="store_true", help="Open interactive MuJoCo viewer")
    args = parser.parse_args()

    ensure_assets()
    environment = SimulationEnvironment()
    controller = PickAndPlaceController(environment, use_sensor=not args.no_sensor)

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
