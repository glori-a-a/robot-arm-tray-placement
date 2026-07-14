#!/usr/bin/env python3
import subprocess, sys
from pathlib import Path
import cv2, mujoco, numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from scripts.generate_soft_cylinder import build_xml, OUTPUT as CYLINDER_XML
from simulation.environment import SimulationEnvironment
from simulation.pick_and_place_controller import PickAndPlaceController

def write_mp4(frames, path, fps=20):
    h, w = frames[0].shape[:2]
    cmd = ["ffmpeg","-y","-f","rawvideo","-vcodec","rawvideo",
           "-s",f"{w}x{h}","-pix_fmt","bgr24","-r",str(fps),"-i","-","-an",
           "-c:v","libx264","-pix_fmt","yuv420p","-movflags","+faststart",str(path)]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    for f in frames:
        p.stdin.write(np.ascontiguousarray(f, dtype=np.uint8).tobytes())
    p.stdin.close()
    if p.wait() != 0:
        raise SystemExit(p.stderr.read().decode())

def write_gif(frames, path, fps=10):
    sampled = frames[::2]
    h, w = sampled[0].shape[:2]
    w, h = w//2, h//2
    cmd = ["ffmpeg","-y","-f","rawvideo","-vcodec","rawvideo",
           "-s",f"{w}x{h}","-pix_fmt","bgr24","-r",str(fps),"-i","-","-an",str(path)]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    for f in sampled:
        small = cv2.resize(f, (w, h))
        p.stdin.write(np.ascontiguousarray(small, dtype=np.uint8).tobytes())
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

    # Fixed camera: shows robot + table + cylinder + tray
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.50, 0.15, 0.42]
    cam.distance = 1.35
    cam.azimuth = 125
    cam.elevation = -28

    ctrl = PickAndPlaceController(env, use_sensor=True)
    frames = []
    def snap():
        renderer.update_scene(env.data, camera=cam)
        frames.append(cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
    for i in range(60):
        mujoco.mj_step(env.model, env.data)
        if i % 3 == 0: snap()
    r = ctrl.run()
    for i in range(180):
        mujoco.mj_step(env.model, env.data)
        if i % 3 == 0: snap()
    ctrl.close(); renderer.close()
    print(f"Captured {len(frames)} frames")
    write_mp4(frames, out / "demo.mp4")
    write_gif(frames, out / "demo.gif")
    print(f"Saved output/demo.mp4 and output/demo.gif")
    print(f"Success: {r['success']}")

if __name__ == "__main__":
    main()
