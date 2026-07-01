#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${VENV_DIR:-$HOME/python_envs/tuthanservo_environment}"

echo "[1/4] Installing Raspberry Pi packages..."
sudo apt update
sudo apt install -y \
    mosquitto \
    mosquitto-clients \
    python3-venv

echo "[2/4] Installing Mosquitto configuration..."
sudo install -m 644 \
    "$PROJECT_DIR/mosquitto/gripper.conf" \
    /etc/mosquitto/conf.d/gripper.conf

echo "[3/4] Enabling and starting Mosquitto..."
sudo systemctl enable --now mosquitto

if ! sudo systemctl is-active --quiet mosquitto; then
    echo "ERROR: Mosquitto could not be started."
    sudo systemctl status mosquitto --no-pager || true
    exit 1
fi

echo "[4/4] Preparing Python environment..."
if [ ! -x "$VENV_DIR/bin/python3" ]; then
    python3 -m venv "$VENV_DIR"
fi

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

python3 -m pip install --upgrade pip
python3 -m pip install -r "$PROJECT_DIR/requirements.txt"

echo
echo "Host setup completed."
echo "Mosquitto will now start automatically at boot."
echo "Daily command:"
echo "  ./run_gripper.sh"
