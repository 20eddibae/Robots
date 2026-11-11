#!/usr/bin/env bash
# Copy our robot code from this laptop to the Jetson.
#
#   ./deploy.sh                  copy code (default robot IP below)
#   ./deploy.sh --build          copy code, then rebuild on the Jetson
#   ./deploy.sh 192.168.0.42     use a different robot IP
#
# Your laptop must be on the robot's Wi-Fi. Asks for the SSH password
# (from Canvas) once per step. Copies:
#   robot/src/team_navigation/  -> ~/a2r-student/robot/src/team_navigation/
#   robot/setup_terminal.sh     -> ~/a2r-student/robot/setup_terminal.sh
# Test images (*.png, *.jpg) and __pycache__ are skipped. Nothing on the
# Jetson is deleted; files are only added or overwritten.
set -euo pipefail

ROBOT_IP=192.168.0.167
BUILD=false
for arg in "$@"; do
    case "$arg" in
        --build) BUILD=true ;;
        -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
        *) ROBOT_IP="$arg" ;;
    esac
done

HOST="a2r-lab-jetson@$ROBOT_IP"
REMOTE="~/a2r-student/robot"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! nc -z -G 3 "$ROBOT_IP" 22 2>/dev/null; then
    echo "Can't reach $ROBOT_IP. Is your laptop on the robot's Wi-Fi, and is that the IP on the OLED?" >&2
    exit 1
fi

echo "==> Copying code to $HOST"
rsync -av \
    --exclude '__pycache__' --exclude '*.png' --exclude '*.jpg' --exclude '.DS_Store' \
    "$REPO/robot/src/team_navigation/" "$HOST:$REMOTE/src/team_navigation/"
rsync -av "$REPO/robot/setup_terminal.sh" "$HOST:$REMOTE/setup_terminal.sh"

if $BUILD; then
    echo "==> Building on the Jetson"
    ssh "$HOST" "bash -c 'source /opt/ros/jazzy/setup.bash && source ~/ros-venv/bin/activate && cd $REMOTE && python -m colcon build --symlink-install'"
    echo "==> Built. Restart terminal B (Ctrl+C, source setup_terminal.sh, relaunch)."
else
    echo "==> Copied. Restart terminal B (Ctrl+C, relaunch). Added new files? Run ./deploy.sh --build"
fi
