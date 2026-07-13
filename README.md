# Robot Arm Tray Placement

Simulated pick-and-place task for a robotics assignment.

A Franka Emika Panda arm picks up a soft cylinder and places it in a rectangular tray. The scene runs in **MuJoCo** with Python control code.

## Task

- Grasp a soft cylinder (~2 cm diameter, 18 cm length)
- Place it in a rectangular tray (~30 × 15 × 0.5 cm)
- Show the full motion in simulation
- **Bonus:** overhead camera finds the cylinder and tray before motion starts

## Quick start

```bash
pip install -r requirements.txt
python scripts/generate_soft_cylinder.py
python main.py
```

Record a demo video:

```bash
python scripts/record_demo.py
```

Output: `output/demo.mp4`

Interactive viewer:

```bash
python main.py --viewer
```

Skip camera detection:

```bash
python main.py --no-sensor
```

## Project layout

```
robot-arm-tray-placement/
├── main.py
├── simulation/
├── sensors/
├── assets/
├── scripts/
├── docs/
└── output/
```

## Tech stack

| Part | Choice |
|------|--------|
| Language | Python 3 |
| Simulator | MuJoCo 3 |
| Robot | Franka Emika Panda (MuJoCo Menagerie URDF/MJCF) |
| Gripper | Parallel finger gripper (built into Panda model) |
| Sensor | Overhead RGB camera + segmentation rendering |
| Motion | Jacobian inverse kinematics |

## Notes

- PyBullet was the first choice but it does not install cleanly on Python 3.12 here. MuJoCo is a common research simulator and works well for this task.
- The cylinder uses many small spheres plus a capsule core. This gives a bumpy, soft-like surface and low stiffness contact.
- Soft grasp is assisted in simulation when finger contact is detected. Real hardware would rely on force control and compliance.

## Author

Individual assignment solution — independent from other projects in this workspace.

## Submission (for Rohan)

See `docs/SUBMISSION_CHECKLIST.md` and `docs/EMAIL_DRAFT.md`.

Quick steps:
1. `./setup.sh`
2. `./push_to_github.sh YOUR_GITHUB_USERNAME`
3. Upload `output/demo.mp4` to Google Drive
4. Email Rohan with repo link + video link + `docs/technical_solution.md`

Submission zip (optional): `/home/ubuntu/robot-arm-tray-placement-submission.zip`

## License

MIT
