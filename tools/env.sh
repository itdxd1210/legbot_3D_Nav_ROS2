#!/usr/bin/env bash
# Source this file from the workspace or any other directory.
legbot_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH=/usr/bin:/bin
source /opt/ros/jazzy/setup.bash
if [[ -f "${legbot_workspace}/install/setup.bash" ]]; then
  source "${legbot_workspace}/install/setup.bash"
fi
export PATH="${legbot_workspace}/.venv-jazzy/bin:${PATH}"
export LEGBOT_DATA_DIR="${legbot_workspace}/data"
export ONNXRUNTIME_ROOT="${legbot_workspace}/third_party/onnxruntime"
# Prefer the project-local diagnostic_updater compatibility library when one is
# provided. Current Jazzy images already ship the required ABI, so the overlay
# is optional rather than a hard startup requirement.
diagnostic_overlay="${legbot_workspace}/.deps/diagnostic_updater_4.2.7/opt/ros/jazzy/lib"
if [[ -f "${diagnostic_overlay}/libdiagnostic_updater.so" ]]; then
  export LD_LIBRARY_PATH="${diagnostic_overlay}:${LD_LIBRARY_PATH:-}"
fi
export LD_LIBRARY_PATH="${legbot_workspace}/third_party/onnxruntime/lib:${legbot_workspace}/third_party/libtorch/lib:${legbot_workspace}/third_party/pct/install/lib:${legbot_workspace}/third_party/Livox-SDK2/install/lib:${legbot_workspace}/third_party/unitree_sdk2/install/lib:${LD_LIBRARY_PATH:-}"
# SDK2 talks to the robot directly; keep ROS processes on the Fast DDS RMW.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset diagnostic_overlay legbot_workspace
