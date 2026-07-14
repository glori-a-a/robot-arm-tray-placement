# Technical Solution

## Overview

This project simulates a robot arm pick-and-place task.

A **Franka Emika Panda** arm picks up a **soft cylinder** and puts it in a **rectangular tray**. Everything runs in **MuJoCo**. The main code is **Python**.

The full flow:

1. Build the scene (robot, table, cylinder, tray)
2. Camera sensor finds object positions
3. Arm moves above the cylinder
4. Gripper closes
5. Arm lifts the cylinder
6. Arm moves to the tray
7. Gripper opens
8. Arm moves away

---

## Tech stack

| Layer | Tool |
|-------|------|
| Language | Python 3.12 |
| Physics sim | MuJoCo 3.10 |
| Robot model | Franka Emika Panda (MuJoCo Menagerie) |
| Math | NumPy, SciPy |
| Vision | OpenCV, MuJoCo segmentation rendering |
| Video | OpenCV VideoWriter |

**Why MuJoCo?** PyBullet was tried first. It failed to build on Python 3.12 in this environment. MuJoCo has pre-built wheels. It is widely used in robotics research. Franka Panda models are easy to get.

---

## Scene design

### Robot

- **Model:** Franka Emika Panda with parallel gripper
- **Source:** [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
- **Control:** 7 arm joints + 1 gripper actuator
- **Motion:** Jacobian-based inverse kinematics ( damped least squares )

### Soft cylinder

| Property | Value |
|----------|-------|
| Diameter | 2 cm |
| Length | 18 cm |
| Surface | Bumpy (48 small spheres + 1 capsule core) |
| Contact | High friction, soft contact parameters |
| Pose | Lying flat on the table (easier to grasp) |

The bumpy surface is not smooth. Soft behavior comes from contact settings and low object mass.

### Rectangular tray

| Property | Value |
|----------|-------|
| Length | 30 cm |
| Width | 15 cm |
| Height | 0.5 cm |
| Extra | Low walls to keep the cylinder inside |

### Environment

- Checkered floor
- Wooden work table (top at 40 cm height)
- Two cameras: overhead (sensor) and side (demo video)

---

## Control flow

```
main.py
  └── SimulationEnvironment      → load MuJoCo scene
  └── PickAndPlaceController
        ├── ObjectDetector       → find cylinder + tray (bonus)
        ├── RobotArm             → IK motion + gripper
        └── attach / release     → carry cylinder during transport
```

### Pick-and-place steps

1. Open gripper
2. Run sensor detection
3. Move above cylinder
4. Move down to grasp height
5. Close gripper
6. Attach cylinder to gripper (contact assist)
7. Lift
8. Move above tray
9. Move down
10. Open gripper
11. Retreat

Success is checked by measuring the XY distance between the cylinder and tray center. Threshold: **10 cm**.

---

## Sensor system (bonus)

**Type:** Overhead RGB camera

**Method:**
1. Render the scene from the overhead camera
2. Use MuJoCo **segmentation rendering** to find geometry IDs
3. Compute the pixel centroid for the cylinder body and tray body
4. Project the pixel onto the table plane (Z = 0.40 m)
5. Use detected XY as motion targets (with sanity check vs default positions)

**Why segmentation?** Color-only detection was unreliable under simulation lighting. Segmentation uses geometry IDs. It is robust and still models a real vision pipeline (detect → localize → act).

Console output example:

```
[sensor] Cylinder found at x=0.558, y=0.049
[sensor] Tray found at x=0.499, y=0.282
```

---

## Files

| File | Role |
|------|------|
| `main.py` | Entry point |
| `simulation/environment.py` | Scene loading |
| `simulation/robot_arm.py` | Arm + IK + gripper |
| `simulation/soft_cylinder.py` | Cylinder constants |
| `simulation/rectangular_tray.py` | Tray constants |
| `simulation/pick_and_place_controller.py` | Task sequence |
| `sensors/object_detector.py` | Camera detection |
| `scripts/generate_soft_cylinder.py` | Build bumpy cylinder MJCF |
| `scripts/record_demo.py` | Save demo video |
| `assets/scene.xml` | Full MuJoCo scene |

---

## How to run

```bash
pip install -r requirements.txt
python scripts/generate_soft_cylinder.py
python main.py
python scripts/record_demo.py
```

Demo video path: [`output/demo.mp4`](https://drive.google.com/file/d/1c7roGvv89SkuV1bBFw05vYxOQRZuegVU/view?usp=sharing)

---

## Assumptions

- Cylinder lies flat on the table (length along X axis)
- Robot base is fixed at the origin
- Grasp uses top-down approach with parallel gripper
- Soft grasp uses simulation contact assist after gripper close
- Object positions are fixed at start (no random spawn)

---

## Possible improvements

- Real physics grasp without attach assist (force control)
- Depth camera + point cloud for noisy detection
- Vertical cylinder side-grasp
- ROS 2 bridge for hardware transfer
- RL-based grasp policy

---

