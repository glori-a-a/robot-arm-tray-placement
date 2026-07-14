# Technical Solution

## Overview

This project simulates a robot-arm pick-and-place task in **MuJoCo** with **Python**.

A **Franka Emika Panda** arm detects a compliant-contact cylinder and a rectangular tray with an overhead camera, approaches with Jacobian IK, validates the grasp, transports the object with a kinematic constraint, then releases it into the tray and evaluates placement.

Closed-loop flow:

camera detection → world-coordinate targets → Jacobian IK approach → grasp validation → constraint-assisted transport → physical (or assisted) release → settling → placement validation

## Tech stack

| Layer | Tool |
|-------|------|
| Language | Python 3 |
| Physics | MuJoCo 3.10 |
| Robot | Franka Emika Panda (MuJoCo Menagerie) |
| Math | NumPy, SciPy |
| Vision | MuJoCo segmentation rendering (+ OpenCV for video) |
| Video | ffmpeg H.264 via `scripts/record_smooth_demo.py` |

**Why MuJoCo?** PyBullet failed to build cleanly on Python 3.12 in this setup. MuJoCo provides wheels, research-grade contact models, and a Menagerie Panda asset.

## Scene objects

### Cylinder (compliant-contact soft approximation)

| Property | Value |
|----------|-------|
| Diameter | 2 cm |
| Length | 18 cm |
| Surface | Non-smooth (capsule core + bumps + localisation beacon) |
| Contact | Soft `solref` / `solimp`, high friction |
| Pose | Lying flat on the table |

This is **not** a FEM / flex deformable body. Softness is approximated by contact compliance and low mass.

### Tray

| Property | Value |
|----------|-------|
| Length × width × height | 30 × 15 × 0.5 cm |

### Robot and cameras

- 7-DoF Panda + parallel gripper
- Overhead RGB camera (offset in +X so the arm does not occlude the cylinder beacon)
- Free camera for demo recording

## Perception

1. Render overhead segmentation
2. Find pixels belonging to the cylinder localisation beacon / tray geoms
3. Project the pixel centroid onto a known plane (beacon height for the cylinder; table plane for the tray)
4. Use detected **XY** as grasp / place targets (preserve Z from known geometry)
5. Reject detections outside the workspace and fall back to the configured scene pose

Detected coordinates **drive** grasp and tray targets when valid.

## Control

### Task states

`INITIALISING → DETECTING → APPROACHING → GRASPING → LIFTING → TRANSPORTING → PLACING → VERIFYING → COMPLETE|FAILED`

### Inner loop

`RobotArm.move_gripper_to` returns whether Cartesian position/orientation tolerances were met (damped least-squares Jacobian IK).

### Grasp assistance

After gripper close, `_validate_grasp_alignment` checks XY/Z alignment and closed command. Only then is a kinematic grasp constraint activated for transport.

This is **not** equivalent to a fully contact-force-driven grasp.

### Placement modes

| Mode | Behaviour |
|------|-----------|
| `physical_release` (default) | Carry above tray, stop syncing, open gripper, settle under gravity/contacts |
| `assisted_release` | Explicit fallback: controlled kinematic lowering into the tray |

### Success evaluation

Checks tray XY bounds (with margin), rest height, and linear/angular speed thresholds. Console prints:

```
[evaluation] inside_x=...
[evaluation] inside_y=...
[evaluation] height_ok=...
[evaluation] stable=...
[result] SUCCESS|FAILED
```

## How to run

```bash
pip install -r requirements.txt
python scripts/generate_soft_cylinder.py
python main.py
python main.py --no-sensor
python main.py --randomize --seed 1
python main.py --placement-mode assisted_release
python scripts/record_smooth_demo.py
python -m pytest -q
```

## Assumptions and limitations

- Soft object = compliant-contact approximation (not FEM)
- Transport uses a validated kinematic grasp constraint
- Perception uses simulator segmentation (not a learned detector)
- Pose is estimated on a known table/beacon plane
- Mild free-joint damping is applied while settling after physical release
- No obstacle avoidance, force/torque feedback, or hardware force control
- Optional `--randomize` is experimental; large offsets can stress IK reachability

## Future work

- Fully contact-driven grasp
- MuJoCo flex / deformable body
- Depth camera + point cloud
- Force/torque feedback
- Trajectory planning / domain randomisation
- Real hardware transfer
