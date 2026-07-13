#!/usr/bin/env bash
# Create a clean repo with ONLY your name in git history.
# Run this on your Ubuntu after you deleted the old GitHub repo and created a new empty one.
#
# Usage:
#   chmod +x fresh_push.sh
#   ./fresh_push.sh

set -euo pipefail
cd "$(dirname "$0")"

REPO_URL="https://github.com/glori-a-a/robot-arm-tray-placement.git"

# Use your real GitHub email if you want (shows on your profile)
git config user.name "glori-a-a"
git config user.email "glori-a-a@users.noreply.github.com"

# Wipe old history and make ONE commit by you
rm -rf .git
git init
git branch -M main
git add -A
git commit -m "Initial submission: robot arm pick-and-place simulation"

git remote add origin "$REPO_URL" 2>/dev/null || git remote set-url origin "$REPO_URL"

echo ""
echo "==> Pushing to $REPO_URL"
echo "    (GitHub will ask you to log in as glori-a-a)"
echo ""
git push -u origin main --force

echo ""
echo "Done! Check: https://github.com/glori-a-a/robot-arm-tray-placement"
echo "Commits should show ONLY glori-a-a."
