"""Small offline frame/pointcloud checks; no Gazebo or full map allocation."""
from pathlib import Path
import runpy
import struct
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField

root = Path(__file__).resolve().parents[1]
helpers = runpy.run_path(str(root / 'src/legbot_bringup/scripts/localization_navigation_bridge'))
# Arbitrary axes, not world-X-only correction; exact six-dimensional composition.
nav_map = np.eye(4)
nav_map[:3, :3] = Rotation.from_euler('xyz', [.15, -.2, .7]).as_matrix()
nav_map[:3, 3] = [2, -3, .4]
map_base = np.eye(4)
map_base[:3, 3] = [5, 6, 1]
odom_base = np.eye(4)
odom_base[:3, :3] = Rotation.from_euler('xyz', [-.2, .1, -.4]).as_matrix()
odom_base[:3, 3] = [-2, 4, .3]
correction = nav_map @ map_base @ np.linalg.inv(odom_base)
assert np.allclose(correction @ odom_base, nav_map @ map_base)
for big in (False, True):
    msg = PointCloud2(height=1, width=2, point_step=16, row_step=36, is_bigendian=big)
    msg.fields = [PointField(name=k, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate(('x', 'y', 'z', 'intensity'))]
    endian = '>' if big else '<'
    msg.data = struct.pack(endian+'8f', 1, 2, 3, 7, 4, 5, 6, 8) + b'PAD!'
    original = bytes(msg.data)
    out = helpers['transform_cloud'](msg, correction)
    values = struct.unpack(endian+'8f', bytes(out.data)[:32])
    assert np.allclose(values[:3], (correction @ [1, 2, 3, 1])[:3])
    assert np.allclose(values[4:7], (correction @ [4, 5, 6, 1])[:3])
    assert values[3] == 7 and values[7] == 8 and bytes(out.data)[32:] == b'PAD!'
    assert bytes(msg.data) == original and out.header.frame_id == 'nav_map'
print('PASS correction composition, arbitrary rotation, cloud fields/padding/byte order')

# Exercise real ROS message callbacks without a simulator or DDS traffic.
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from types import SimpleNamespace
from collections import OrderedDict

Bridge = helpers['NavigationBridge']
clock = rclpy.clock.Clock()
received = []
tf = TransformStamped()
tf.transform.rotation.w = 1.0
bridge = SimpleNamespace(nav_map=np.eye(4), correction=np.eye(4),
    pending=OrderedDict(), statuses=OrderedDict(), last_stamp=-1,
    accepted=0, dropped=0, last_accept_time=None,
    get_clock=lambda: clock,
    buffer=SimpleNamespace(lookup_transform=lambda *args: tf),
    trim=Bridge.trim, stamp=Bridge.stamp,
    odom_publishers={'base': SimpleNamespace(publish=received.append)},
    broadcaster=SimpleNamespace(sendTransform=lambda msg: None))
match = PoseWithCovarianceStamped()
match.header.frame_id = 'localization_probe'
match.header.stamp = clock.now().to_msg()
match.pose.pose.orientation.w = 1.0
match.pose.pose.position.z = .2
Bridge.matched(bridge, match)
Bridge.update(bridge)
assert bridge.accepted == 0  # Unverified pose/initial seed cannot correct control.
status = DiagnosticArray()
status.header = match.header
status.status = [DiagnosticStatus(name='lidar_localization_ros2/alignment', message='rejected')]
Bridge.status(bridge, status)
Bridge.update(bridge)
assert bridge.accepted == 0 and np.allclose(bridge.correction, np.eye(4))
Bridge.matched(bridge, match)
status.status[0].message = 'ok'
Bridge.status(bridge, status)
Bridge.update(bridge)
assert bridge.accepted == 1 and abs(bridge.correction[2, 3] - .2) < 1e-9
raw = Odometry()
raw.header.frame_id = 'odom'
raw.pose.pose.orientation.w = 1.0
raw.twist.twist.linear.x = .4
Bridge.odom(bridge, raw, 'base')
assert raw.pose.pose.position.z == 0
assert received[-1].header.frame_id == 'nav_map'
assert received[-1].pose.pose.position.z == .2
assert received[-1].twist.twist.linear.x == .4
# Missing exact-time TF must hold the previous correction.
match.header.stamp = clock.now().to_msg()
match.pose.pose.position.z = 2.
status.header = match.header
Bridge.matched(bridge, match)
Bridge.status(bridge, status)
def missing(*args):
    raise RuntimeError('TF missing')
bridge.buffer.lookup_transform = missing
Bridge.update(bridge)
assert bridge.accepted == 1 and abs(bridge.correction[2, 3] - .2) < 1e-9
print('PASS accepted-only correction, rejected/missing TF hold, raw odom preservation, body twist')
