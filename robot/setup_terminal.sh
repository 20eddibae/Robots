# Prepare a Jetson terminal for RUNNING the robot (docs/SETUP.md,
# "Prepare each Jetson terminal"). Must be sourced, not executed:
#
#   source ~/a2r-student/robot/setup_terminal.sh
#
# Do not source this in a terminal you will BUILD in: build in a fresh
# terminal, then open new terminals and source this.

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "Run this with 'source', not 'bash':  source ${0}" >&2
    exit 1
fi

_a2r_install="$HOME/a2r-student/robot/install/setup.bash"
if [ ! -f "$_a2r_install" ]; then
    echo "Not built yet: $_a2r_install is missing. Build in ~/a2r-student/robot first." >&2
    unset _a2r_install
    return 1
fi

source /opt/ros/jazzy/setup.bash
source "$_a2r_install"
source ~/ros-venv/bin/activate
source /etc/a2r-robot/robot.sh
unset _a2r_install

printf 'robot=%s domain=%s\n' "$ROBOT_NAMESPACE" "$ROS_DOMAIN_ID"
ros2 pkg prefix a2r_robot
