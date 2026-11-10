#!/usr/bin/env bash

# Intended to be sourced from Bash.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Use: source tools/activate_physical_env.sh"
    exit 1
fi

_SAPS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Prevent user-installed ZED Python dependencies from overriding
# the system NumPy/SciPy versions used by SAPS and ROS 2.
export PYTHONNOUSERSITE=1

deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true

source /opt/ros/jazzy/setup.bash
export FRANKA_ROS2_WS="$HOME/franka_ehsan_ros2_ws"
export FRANKA_ROS2_INSTALL="$FRANKA_ROS2_WS/install"
export FRANKA_DESCRIPTION_DIR="$FRANKA_ROS2_WS/src/franka_description"
export IGD_FR3_CONTROL_DIR="$FRANKA_ROS2_WS/src/igd_fr3_control"
source "$FRANKA_ROS2_INSTALL/setup.bash"
export FRANKA_DROID_INSTALL="${FRANKA_DROID_INSTALL:-$FRANKA_ROS2_WS/install-droid-current}"
source "$FRANKA_DROID_INSTALL/setup.bash" || return 1
source "$_SAPS_ROOT/.venv-physical/bin/activate"

export PHYSICAL_PYTHON="$_SAPS_ROOT/.venv-physical/bin/python"

export PHYSICAL_CONFIG="$_SAPS_ROOT/configs/local/zed_robotiq.json"

export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$_SAPS_ROOT/src:$_SAPS_ROOT/third_party/openpi/packages/openpi-client/src:${PYTHONPATH:-}"

echo "SAPS physical environment initialized."
echo "Python: $PHYSICAL_PYTHON"

unset _SAPS_ROOT
