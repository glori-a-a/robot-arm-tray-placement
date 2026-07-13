"""Record a demo video of the pick-and-place task."""

import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.generate_soft_cylinder import build_xml, OUTPUT as CYLINDER_XML
from simulation.environment import SimulationEnvironment
from simulation.pick_and_place_controller import PickAndPlaceController


def ensure_assets() -> None:
    if not CYLINDER_XML.exists():
        CYLINDER_XML.parent.mkdir(parents=True, exist_ok=True)
        CYLINDER_XML.write_text(build_xml())


def main() -> None:
    ensure_assets()
    output_dir = ROOT / "output"
    output_dir.mkdir(exist_ok=True)
    video_path = output_dir / "demo.mp4"

    environment = SimulationEnvironment()
    model = environment.model
    data = environment.data

    renderer = mujoco.Renderer(model, height=720, width=1280)
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "demo_camera")

    controller = PickAndPlaceController(environment, use_sensor=True)

    frames: list[np.ndarray] = []

    def capture_frame() -> None:
        renderer.update_scene(data, camera=camera_id)
        frame = renderer.render()
        frames.append(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

    # Record a short warm-up
    for _ in range(60):
        mujoco.mj_step(model, data)
        if _ % 3 == 0:
            capture_frame()

    result = controller.run()

    for _ in range(180):
        mujoco.mj_step(model, data)
        if _ % 3 == 0:
            capture_frame()

    controller.close()
    renderer.close()

    if not frames:
        raise RuntimeError("No frames captured")

    height, width = frames[0].shape[:2]
    writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        20.0,
        (width, height),
    )
    for frame in frames:
        writer.write(frame)
    writer.release()

    print(f"Saved demo video to {video_path}")
    print(f"Task success: {result['success']}")


if __name__ == "__main__":
    main()
