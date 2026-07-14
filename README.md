# Robot Arm Tray Placement

A Franka Emika Panda arm picks up a soft cylinder and places it in a rectangular tray. The scene runs in **MuJoCo** with Python control code.

## Demo

Full pick-and-place recording (approach → grasp → pick → place):

**[Watch on Google Drive](https://drive.google.com/file/d/1c7roGvv89SkuV1bBFw05vYxOQRZuegVU/view?usp=sharing)**

## Features

- Soft cylinder (~2 cm diameter, 18 cm length), non-smooth surface
- Rectangular tray (~30 × 15 × 0.5 cm)
- Full motion: approach → grasp → pick → place
- Overhead camera object detection (bonus)

## Quick start

```bash
pip install -r requirements.txt
python scripts/generate_soft_cylinder.py
python main.py
```

Record a demo video:

```bash
python scripts/record_smooth_demo.py
```

Output: `output/demo.mp4`

```bash
python main.py --viewer
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
| Robot | Franka Emika Panda (MuJoCo Menagerie) |
| Gripper | Parallel finger gripper |
| Sensor | Overhead RGB camera + segmentation |
| Motion | Jacobian inverse kinematics |

## Notes

- PyBullet did not install cleanly on Python 3.12 in this setup, so MuJoCo is used instead.
- The cylinder uses a capsule core plus small surface bumps (non-smooth) with soft contact parameters.
- Grasp in simulation uses contact assist after the gripper closes; real hardware would use force control / compliance.

## License

MIT
