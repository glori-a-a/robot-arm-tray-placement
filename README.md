# Robot Arm Tray Placement

A Franka Emika Panda arm picks up a compliant-contact cylindrical object and places it in a rectangular tray. The scene runs in **MuJoCo** with Python control code.

## Demo

Full pick-and-place recording (approach → grasp → pick → place):

**[Watch on Google Drive][(https://drive.google.com/file/d/1c7roGvv89SkuV1bBFw05vYxOQRZuegVU/view?usp=sharing)]**

## Features

- Compliant-contact cylinder (~2 cm diameter, 18 cm length), non-smooth surface
- Rectangular tray (~30 × 15 × 0.5 cm)
- Closed loop: overhead segmentation → world XY → Jacobian IK → grasp validation → transport → release → placement checks
- Bonus: overhead camera object detection drives grasp and tray targets

## Quick start

```bash
pip install -r requirements.txt
python scripts/generate_soft_cylinder.py
python main.py
```

Useful modes:

```bash
python main.py --viewer
python main.py --no-sensor
python main.py --randomize --seed 1
python main.py --placement-mode assisted_release
```

Record a demo video:

```bash
python scripts/record_smooth_demo.py
```

Output: `output/demo.mp4`

## Project layout

```
robot-arm-tray-placement/
├── main.py
├── simulation/
├── sensors/
├── assets/
├── scripts/
├── docs/
├── tests/
└── output/
```

## Tech stack

| Part | Choice |
|------|--------|
| Language | Python 3 |
| Simulator | MuJoCo 3 |
| Robot | Franka Emika Panda (MuJoCo Menagerie) |
| Gripper | Parallel finger gripper |
| Sensor | Overhead RGB + segmentation |
| Motion | Jacobian inverse kinematics |

## Modelling notes

- The cylinder is a **compliant-contact approximation** of a soft, non-smooth object (rigid capsule + bumps + soft MuJoCo contacts). It is **not** a finite-element deformable body.
- After gripper closure and grasp-alignment validation, a **kinematic grasp constraint** stabilises transport. This is an engineering trade-off, not a fully force-driven grasp.
- Default placement uses **physical_release** (break constraint, open gripper, settle under gravity). `assisted_release` is an explicit fallback.

## Perception loop

overhead segmentation → pixel centroid → table/beacon-plane projection → world XY → IK control

Detected coordinates drive both grasp and tray targets (with workspace sanity checks and configured-pose fallback).

## License

MIT
