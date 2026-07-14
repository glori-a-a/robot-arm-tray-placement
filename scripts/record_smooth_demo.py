#!/usr/bin/env python3
"""
Smooth demo video — hooks every physics step so motion is visible.
Run: python3 scripts/record_smooth_demo.py
No other files need to be changed.
"""

import subprocess
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

FRAME_INTERVAL = 8  # ~20 fps


def write_mp4(frames, path, fps=20):
    h, w = frames[0].shape[:2]
    cmd = [
        "ffmpeg", "-y", "-f", "rawvideo", "-vcodec", "rawvideo",
        "-s", f"{w}x{h}", "-pix_fmt", "bgr24", "-r", str(fps),
        "-i", "-", "-an",
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(path),
    ]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    for f in frames:
        p.stdin.write(np.ascontiguousarray(f, dtype=np.uint8).tobytes())
    p.stdin.close()
    if p.wait() != 0:
        raise SystemExit(p.stderr.read().decode())


def main():
    if not CYLINDER_XML.exists():
        CYLINDER_XML.parent.mkdir(parents=True, exist_ok=True)
        CYLINDER_XML.write_text(build_xml())

    out = ROOT / "output"
    out.mkdir(exist_ok=True)

    env = SimulationEnvironment()
    renderer = mujoco.Renderer(env.model, 720, 1280)

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.50, 0.15, 0.42]
    cam.distance = 1.35
    cam.azimuth = 125
    cam.elevation = -28

    frames = []
    step_count = 0
    original_step = mujoco.mj_step

    def capture():
        renderer.update_scene(env.data, camera=cam)
        frames.append(cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))

    def hooked_step(model, data):
        nonlocal step_count
        original_step(model, data)
        step_count += 1
        if step_count % FRAME_INTERVAL == 0:
            capture()

    capture()
    mujoco.mj_step = hooked_step

    try:
        ctrl = PickAndPlaceController(env, use_sensor=True)
        result = ctrl.run()
        for _ in range(80):
            hooked_step(env.model, env.data)
        ctrl.close()
    finally:
        mujoco.mj_step = original_step

    renderer.close()

    mp4 = out / "demo.mp4"
    write_mp4(frames, mp4)
    print(f"Captured {len(frames)} frames ({len(frames)/20:.1f}s)")
    print(f"Saved: {mp4}")
    print(f"Success: {result['success']}")
    print("Play: vlc output/demo.mp4")


if __name__ == "__main__":
    main()
