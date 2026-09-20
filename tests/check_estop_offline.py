"""Static emergency-stop contract check; starts no ROS node or robot connection."""
import os
from pathlib import Path

controller_header = Path(
    'src/rl_quadruped_controller/src/RlQuadrupedController.h').read_text()
controller_source = Path(
    'src/rl_quadruped_controller/src/RlQuadrupedController.cpp').read_text()
passive_source = Path('src/controller_common/src/FSM/StatePassive.cpp').read_text()
keyboard_source = Path('src/legbot_bringup/scripts/go2_estop_keyboard').read_text()
keyboard_path = Path('src/legbot_bringup/scripts/go2_estop_keyboard')
bringup_cmake = Path('src/legbot_bringup/CMakeLists.txt').read_text()
real_launch = Path('src/legbot_bringup/launch/real.launch.py').read_text()
hardware_header = Path(
    'src/hardware_unitree_sdk2/include/hardware_unitree_sdk2/HardwareUnitree.h').read_text()
hardware_source = Path('src/hardware_unitree_sdk2/src/HardwareUnitree.cpp').read_text()
policy_source = Path('src/rl_quadruped_controller/src/FSM/StateRL.cpp').read_text()
history_source = Path(
    'src/rl_quadruped_controller/src/common/ObservationBuffer.cpp').read_text()

assert 'std::atomic_bool emergency_stop_latched_{false}' in controller_header
assert '"/go2/emergency_stop"' in controller_source
assert 'transient_local()' in controller_source
assert 'emergency_stop_latched_.load' in controller_source
assert 'FSMStateName::PASSIVE' in controller_source
assert 'restart real.launch.py to clear it' in controller_source
assert 'set_value(0.0)' in passive_source
assert 'set_value(1.0)' in passive_source
assert "b' '" in keyboard_source and "b'\\x1b'" in keyboard_source
assert "b'e'" not in keyboard_source and "b'E'" not in keyboard_source
assert "'/go2/emergency_stop'" in keyboard_source
assert "'/go2/control_mode'" in keyboard_source
assert "declare_parameter('max_vx', 0.10)" in keyboard_source
assert "key == b'1'" in keyboard_source
assert "key == b'2'" in keyboard_source
assert "key == b'3'" in keyboard_source
assert "b'w'" in keyboard_source and "b's'" in keyboard_source
assert "b'x'" in keyboard_source
assert 'Speed key ignored until key 3' in keyboard_source
assert 'DurabilityPolicy.TRANSIENT_LOCAL' in keyboard_source
assert 'go2_estop_keyboard' in bringup_cmake
assert os.access(keyboard_path, os.X_OK)
assert "DeclareLaunchArgument('hardware_update_rate', default_value='400')" in real_launch
assert "'update_rate': int(val('hardware_update_rate'))" in real_launch
assert 'on_activate(const rclcpp_lifecycle::State &previous_state)' in hardware_header
assert 'state_received_cv_.wait_for' in hardware_source
assert 'Fresh GO2 LowState received; hardware activation is safe' in hardware_source
assert 'refusing to activate hardware' in hardware_source
assert 'GO2 policy warm-up inference complete' in policy_source
assert 'First live RL inference accepted' in policy_source
assert 'First RL sample: quaternion=' not in policy_source
assert 'std::memmove' in history_source and 'std::memcpy' in history_source
assert '.clone(' not in history_source and 'torch::cat' not in history_source
compile(keyboard_source, 'go2_estop_keyboard', 'exec')

print('PASS latched GO2 keyboard emergency stop -> passive damping contract')
