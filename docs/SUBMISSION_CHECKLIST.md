# Submission Checklist

Deadline: **Tuesday 14 July 2026, 23:59 UTC**

## Before you email Rohan

- [ ] Run `./setup.sh` — simulation succeeds
- [ ] Run `python3 scripts/record_demo.py` — `output/demo.mp4` is fresh
- [ ] Push code to GitHub (`./push_to_github.sh YOUR_USERNAME`)
- [ ] Upload `output/demo.mp4` to Google Drive or Dropbox
- [ ] Copy share link (anyone with link can view)
- [ ] Send email using `docs/EMAIL_DRAFT.md` as template
- [ ] Attach or link: technical doc + repo + video

## What to send

| Item | File / Link |
|------|-------------|
| Technical doc | `docs/technical_solution.md` or PDF |
| Code | GitHub repo link |
| Demo video | `output/demo.mp4` → cloud link |

## Quick test commands

```bash
cd robot-arm-tray-placement
./setup.sh
python3 main.py --no-sensor   # without camera
python3 main.py               # with camera (bonus)
python3 main.py --viewer      # 3D viewer (needs display)
```

## Create submission zip (optional)

```bash
cd ..
zip -r robot-arm-tray-placement-submission.zip robot-arm-tray-placement \
  -x "*/.git/*" "*/__pycache__/*" "*/output/*.png"
```

Include `output/demo.mp4` in the zip if email attachment size allows.

## Interview prep

See conversation notes: 30-second pitch, architecture, IK, sensor, limitations.
