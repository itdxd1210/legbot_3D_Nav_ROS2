#!/usr/bin/env bash
# Source in a fresh host terminal for the isolated native Jazzy build.
legbot_host_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f "${legbot_host_workspace}/install/host-jazzy/setup.bash" ]]; then
  echo "Native build missing: ${legbot_host_workspace}/install/host-jazzy/setup.bash" >&2
  unset legbot_host_workspace
  return 1
fi

source /opt/ros/jazzy/setup.bash
source "${legbot_host_workspace}/install/host-jazzy/setup.bash"

host_ros_overlay="${legbot_host_workspace}/.deps/host-jazzy/opt/ros/jazzy"
export AMENT_PREFIX_PATH="${host_ros_overlay}:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="${host_ros_overlay}:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${host_ros_overlay}/lib:${legbot_host_workspace}/third_party/onnxruntime/lib:${legbot_host_workspace}/third_party/libtorch/lib:${legbot_host_workspace}/third_party/pct/install/lib:${legbot_host_workspace}/third_party/Livox-SDK2/install/lib:${legbot_host_workspace}/third_party/unitree_sdk2/install/lib:${LD_LIBRARY_PATH:-}"
export ONNXRUNTIME_ROOT="${legbot_host_workspace}/third_party/onnxruntime"
export PCT_PYTHON="${legbot_host_workspace}/.venv-jazzy/bin/python3"
export LEGBOT_DATA_DIR="${legbot_host_workspace}/data"
export ROS_LOG_DIR="${legbot_host_workspace}/log/host-jazzy/ros"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

unset host_ros_overlay legbot_host_workspace
