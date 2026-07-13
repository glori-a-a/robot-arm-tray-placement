# Email Draft for Rohan

**Subject:** Technical Assignment Submission — Robot Arm Tray Placement (Xinyue)

---

Hi Rohan,

Please find my submission for the robotic pick-and-place assignment below.

## 1. Technical solution document

Attached: `technical_solution.pdf` (or see `docs/technical_solution.md` in the repo)

**Summary:**
- **Simulator:** MuJoCo 3
- **Robot:** Franka Emika Panda with parallel gripper
- **Task:** Grasp soft cylinder (2 cm × 18 cm) → place in rectangular tray (30 × 15 × 0.5 cm)
- **Sensor (bonus):** Overhead RGB camera with segmentation-based object detection
- **Control:** Jacobian inverse kinematics + pick-and-place state machine

## 2. Working project

**GitHub (download code here):**

https://github.com/glori-a-a/MMSI/tree/robot-arm-tray-placement

```bash
git clone -b robot-arm-tray-placement https://github.com/glori-a-a/MMSI.git robot-arm-tray-placement
```
chmod +x setup.sh
./setup.sh
```

## 3. Demo video

**Google Drive / Dropbox link:** [PASTE YOUR VIDEO LINK HERE]

Local file: `output/demo.mp4`

The video shows: scene setup → sensor detection → approach → grasp → lift → transport → place → release.

---

Happy to walk through the live demo in the follow-up meeting.

Best regards,
Xinyue
