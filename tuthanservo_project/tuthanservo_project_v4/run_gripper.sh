#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${VENV_DIR:-$HOME/python_envs/tuthanservo_environment}"
BROKER="${BROKER:-localhost}"

if ! systemctl is-active --quiet mosquitto; then
    echo "Mosquitto is not active. Starting it..."
    sudo systemctl start mosquitto
fi

if ! systemctl is-active --quiet mosquitto; then
    echo "ERROR: Mosquitto is not running."
    sudo systemctl status mosquitto --no-pager || true
    exit 1
fi

if [ ! -x "$VENV_DIR/bin/python3" ]; then
    echo "ERROR: Python environment was not found:"
    echo "  $VENV_DIR"
    echo "Run ./setup_host.sh first."
    exit 1
fi

# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

echo "Project: $PROJECT_DIR"
echo "Broker:  $BROKER"
echo

exec python3 \
    "$PROJECT_DIR/raspberry_pi/gripper_client.py" \
    --broker "$BROKER" \
    "$@"
