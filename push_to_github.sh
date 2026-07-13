#!/usr/bin/env bash
# Push this project to your GitHub account.
# Usage: ./push_to_github.sh YOUR_GITHUB_USERNAME

set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: ./push_to_github.sh YOUR_GITHUB_USERNAME"
  echo "Example: ./push_to_github.sh xinyue-zhang"
  exit 1
fi

USERNAME="$1"
REPO_NAME="robot-arm-tray-placement"
REMOTE="https://github.com/${USERNAME}/${REPO_NAME}.git"

cd "$(dirname "$0")"

if ! git remote get-url origin &>/dev/null; then
  git remote add origin "$REMOTE"
else
  git remote set-url origin "$REMOTE"
fi

echo "==> Create repo on GitHub first (if not exists):"
echo "    https://github.com/new"
echo "    Name: ${REPO_NAME}"
echo "    Public, no README/license (we already have them)"
echo ""
read -p "Press Enter after you created the empty repo on GitHub..."

echo "==> Pushing to ${REMOTE}"
git push -u origin main

echo ""
echo "Done! Repo URL: https://github.com/${USERNAME}/${REPO_NAME}"
