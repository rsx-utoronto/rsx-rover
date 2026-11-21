#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing system venv package & granting camera permissions..."
sudo apt update && sudo apt install -y python3-venv
sudo usermod -aG video "$USER"

echo "Creating Python virtual environment..."
python3 -m venv --system-site-packages "$SCRIPT_DIR/.venv"
"$SCRIPT_DIR/.venv/bin/pip" install --upgrade pip
"$SCRIPT_DIR/.venv/bin/pip" install -r "$SCRIPT_DIR/requirements.txt"

echo "Done! Log out and back in once if this is a fresh machine so 'video' group permissions take effect."
