#!/usr/bin/env bash
# One-command setup for Ubuntu
set -euo pipefail

cd "$(dirname "$0")"

echo "==> Installing Python dependencies..."
pip3 install -r requirements.txt

echo "==> Generating soft cylinder asset..."
python3 scripts/generate_soft_cylinder.py

echo "==> Running simulation (headless)..."
python3 main.py

echo ""
echo "Done! Next steps:"
echo "  python3 main.py --viewer     # interactive 3D viewer"
echo "  python3 scripts/record_demo.py  # save output/demo.mp4"
