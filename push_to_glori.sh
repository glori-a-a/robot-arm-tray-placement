#!/usr/bin/env bash
# Push to glori-a-a/robot-arm-tray-placement
set -euo pipefail
cd "$(dirname "$0")"

REMOTE="https://github.com/glori-a-a/robot-arm-tray-placement.git"

if ! git remote get-url origin &>/dev/null; then
  git remote add origin "$REMOTE"
else
  git remote set-url origin "$REMOTE"
fi

echo "============================================"
echo " Step 1: Create empty repo on GitHub"
echo "============================================"
echo ""
echo " Open: https://github.com/new"
echo " Owner: glori-a-a"
echo " Name:  robot-arm-tray-placement"
echo " Public, NO README, NO .gitignore, NO license"
echo ""
read -p "Press Enter after you created the empty repo..."

echo ""
echo "==> Pushing..."
git push -u origin main

echo ""
echo "Done! https://github.com/glori-a-a/robot-arm-tray-placement"
