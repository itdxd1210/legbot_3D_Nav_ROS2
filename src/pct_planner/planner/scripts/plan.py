import sys
import argparse
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import remove_ros_args
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init()
node = rclpy.create_node('pct_planner')
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

from utils import *
from planner_wrapper import TomogramPlanner


from geometry_msgs.msg import Point
from visualization_msgs.msg import *
from geometry_msgs.msg import PointStamped
from interactive_markers.menu_handler import *
from interactive_markers.interactive_marker_server import *

sys.path.append('../')
from config import Config

parser = argparse.ArgumentParser()
parser.add_argument('--scene', type=str, default='Go2', help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\']')
parser.add_argument('--tomogram', type=str, default='', help='Tomogram basename without .pickle')
parser.add_argument('--start', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
parser.add_argument('--start-mode', choices=('fixed', 'current'), default='fixed')
parser.add_argument('--start-pose-topic', default='')
parser.add_argument('--start-pose-frame', default='initial_map')
parser.add_argument('--map-from-pct', type=float, nargs=7,
                    help='Saved map <- PCT rigid transform: xyz quaternion xyzw')
parser.add_argument('--goal', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
parser.add_argument('--path-topic', default='/pct_path')
parser.add_argument('--frame-id', default='building_pct')
parser.add_argument('--display-frame', default='',
                    help='RViz fixed frame for the one-time map-cloud republish')
parser.add_argument('--optimize', action='store_true',
                    help='Use experimental GPMP optimization instead of the stable PCT A* route')
args = parser.parse_args(remove_ros_args(sys.argv)[1:])
if args.start_mode == 'current':
    if not args.start_pose_topic or args.map_from_pct is None or args.start is not None:
        parser.error('current start requires pose topic/map_from_pct and forbids --start fallback')
    transform = np.asarray(args.map_from_pct)
    if (not np.isfinite(transform).all()
            or abs(np.linalg.norm(transform[3:]) - 1.) > 1e-3):
        parser.error('--map-from-pct must be finite with a normalized quaternion')

display_frame = args.display_frame or args.frame_id
display_tf_buffer = Buffer(node=node) if display_frame != args.frame_id else None
display_tf_listener = TransformListener(display_tf_buffer, node) if display_tf_buffer else None
cfg = Config()


scene_defaults = {
    'Go2': ('building2_9', [-5.5, 6.0, 0.5], [2.0, -3.0, 4.5]),
}
if args.scene not in scene_defaults and not (args.tomogram and args.start and args.goal):
    parser.error('unknown scene or missing --tomogram/--start/--goal for a custom map')

default_tomogram, default_start, default_goal = scene_defaults.get(
    args.scene, (args.tomogram, args.start, args.goal)
)
tomo_file = args.tomogram or default_tomogram
start_pos = (np.asarray(args.start or default_start, dtype=np.float32)
             if args.start_mode == 'fixed' else None)
end_pos = np.asarray(args.goal or default_goal, dtype=np.float32)
if ((start_pos is not None and not np.isfinite(start_pos).all())
        or not np.isfinite(end_pos).all()):
    parser.error('--start and --goal must contain finite coordinates')

path_pub = node.create_publisher(
    Path, args.path_topic,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
cloud_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
global_points_pub = node.create_publisher(PointCloud2, '/global_points', cloud_qos)
tomogram_pub = node.create_publisher(PointCloud2, '/tomogram', cloud_qos)
planner = TomogramPlanner(cfg)

POINT_FIELDS_XYZI = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]

# 新增全局变量
last_planned_start_pos = None
last_planned_end_pos = None
plan_timer = None
cloud_timer = None
map_cloud_messages = None
PLAN_INTERVAL = 0.5  # 规划检查间隔（秒）
current_start_msg = None
current_plan_failed = False


def pct_start_from_odometry(msg, map_from_pct, expected_frame, now):
    """Accepted current base pose -> PCT coordinates; no height adjustment."""
    if msg.header.frame_id != expected_frame or msg.child_frame_id != 'base':
        raise ValueError('Expected accepted map-frame base odometry')
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if not 0. <= now - stamp <= 0.5:
        raise ValueError('Current base pose is stale or future-dated')
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    values = np.asarray([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    if not np.isfinite(values).all() or not 0.5 <= np.linalg.norm(values[3:]) <= 1.5:
        raise ValueError('Invalid current base pose')
    tx, ty, tz, x, y, z, w = map_from_pct
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    return rotation.T @ (values[:3] - [tx, ty, tz])


def check_current_start_terrain(candidate, planner):
    """Reject uncovered starts before the existing layer selector/search."""
    index = planner.pos2idx(candidate[:2])
    # pos2idx returns [column, row]; stored grids use [row, column].
    if np.any(index < 0) or np.any(index >= np.asarray(planner.map_dim)[::-1]):
        raise ValueError('Current PCT start is outside the Tomogram')
    layer = planner.match_best_layer(*candidate)
    column, row = index.astype(int)
    ground = planner.layer_elev_grids[layer, row, column]
    cost = planner.tomogram[0, layer, row, column]
    if (not np.isfinite(ground) or ground <= -99. or not np.isfinite(cost) or cost <= 0.
            or abs(ground-candidate[2]) > planner.slice_dh):
        raise ValueError('Current PCT start has no height-compatible mapped terrain')


def on_current_start(msg):
    global current_start_msg
    if start_pos is not None or current_plan_failed:
        return
    try:
        pct_start_from_odometry(msg, args.map_from_pct, args.start_pose_frame,
                                node.get_clock().now().nanoseconds * 1e-9)
    except ValueError:
        return
    current_start_msg = msg


def publish_map_clouds():
    """Rebuild RViz point clouds from the bundled tomogram.

    The repository's .pcd files may be Git LFS pointer files, while the
    precomputed tomogram contains the terrain surfaces needed by PCT.  Both
    clouds are latched so RViz can start after this node and still receive
    them.
    """
    elevations = planner.layer_elev_grids
    traversability = planner.tomogram[0]
    xy = planner.grid_xy_coords

    # Show every valid layer as a PCD-like surface cloud.  Z is also used as
    # intensity so RViz can apply the same height colouring as the raw PCD.
    global_chunks = []
    for layer in range(planner.n_slice):
        z = elevations[layer]
        valid = np.isfinite(z) & (z > -99.0)
        if np.any(valid):
            xyz = np.column_stack((xy[valid], z[valid]))
            global_chunks.append(np.column_stack((xyz, z[valid])))
    global_points = np.vstack(global_chunks).astype(np.float32, copy=False)

    # Match tomography.py's visualisation: suppress a lower layer wherever
    # the next layer describes the same surface, leaving distinct floors and
    # stair transitions visible.  Traversability is the intensity channel.
    vis_elev = elevations.copy()
    for layer in range(planner.n_slice - 1):
        both_valid = (vis_elev[layer] > -99.0) & (vis_elev[layer + 1] > -99.0)
        same_surface = both_valid & (
            (vis_elev[layer + 1] - vis_elev[layer]) < planner.slice_dh)
        vis_elev[layer, same_surface] = -100.0

    tomo_chunks = []
    for layer in range(planner.n_slice):
        z = vis_elev[layer]
        valid = np.isfinite(z) & (z > -99.0) & np.isfinite(traversability[layer])
        if np.any(valid):
            xyz = np.column_stack((xy[valid], z[valid]))
            tomo_chunks.append(np.column_stack((xyz, traversability[layer][valid])))
    tomogram_points = np.vstack(tomo_chunks).astype(np.float32, copy=False)

    header = Header()
    header.stamp = node.get_clock().now().to_msg()
    header.frame_id = args.frame_id
    global map_cloud_messages
    map_cloud_messages = (
        pc2.create_cloud(header, POINT_FIELDS_XYZI, global_points),
        pc2.create_cloud(header, POINT_FIELDS_XYZI, tomogram_points),
    )
    republish_map_clouds()
    node.get_logger().info(
        'Published reconstructed PCT map clouds: /global_points=%d, /tomogram=%d, frame=%s' %
        (len(global_points), len(tomogram_points), args.frame_id))


def republish_map_clouds():
    """Publish the two cached map clouds without rebuilding them."""
    if map_cloud_messages is None:
        return
    stamp = node.get_clock().now().to_msg()
    map_cloud_messages[0].header.stamp = stamp
    map_cloud_messages[1].header.stamp = stamp
    global_points_pub.publish(map_cloud_messages[0])
    tomogram_pub.publish(map_cloud_messages[1])


def publish_clouds_when_rviz_is_ready():
    """Send one live copy after RViz matches and its fixed-frame TF exists."""
    rviz_matched = (
        global_points_pub.get_subscription_count() > 0 and
        tomogram_pub.get_subscription_count() > 0)
    if not rviz_matched:
        return
    if display_tf_buffer is not None and not display_tf_buffer.can_transform(
            display_frame, args.frame_id, Time()):
        return
    republish_map_clouds()
    cloud_timer.cancel()
    node.get_logger().info(
        'PCT map cloud late-join publish complete: display_frame=%s' % display_frame)


def plan_callback():  # 关键修改：添加event参数接收TimerEvent
    """定时器回调函数：执行路径规划并发布"""
    global last_planned_end_pos, last_planned_start_pos,end_pos, start_pos, plan_timer
    global current_plan_failed
    if args.start_mode == 'current':
        if current_plan_failed:
            return
        if start_pos is None:
            if current_start_msg is None:
                return
            try:
                candidate = pct_start_from_odometry(
                    current_start_msg, args.map_from_pct, args.start_pose_frame,
                    node.get_clock().now().nanoseconds * 1e-9)
            except ValueError:
                return  # Wait for the next fresh accepted pose, never a default start.
            try:
                check_current_start_terrain(candidate, planner)
            except ValueError as exc:
                current_plan_failed = True
                node.get_logger().error('PCT_START_REJECTED: %s; navigation remains blocked' % exc)
                return
            start_pos = candidate.astype(np.float32)
            node.get_logger().info('PCT_START_SELECTED mode=current frame=%s xyz=%s' %
                                   (args.frame_id, start_pos.tolist()))
            # Once selected, keep this mission's start fixed, not moving with odometry.
            make6DofMarker(start_pos, 'start_pos', show_6dof=False)

    # 检查位置是否发生变化（考虑浮点数精度）
    position_changed = True
    if last_planned_start_pos is not None and last_planned_end_pos is not None:
        if np.linalg.norm(end_pos - last_planned_end_pos) < 0.01 and \
              np.linalg.norm(start_pos - last_planned_start_pos) < 0.01 :
            position_changed = False
    
    # 只有位置变化时才执行规划
    if position_changed:
        try:
            traj_3d = planner.plan(start_pos, end_pos, optimize=args.optimize)
            if traj_3d is not None:
                stamp = node.get_clock().now().to_msg()
                path_pub.publish(traj2ros(traj_3d, args.frame_id, stamp))
                print(
                    f"规划并发布PCT路径: points={len(traj_3d)}, "
                    f"start_pos={start_pos}, end_pos={end_pos}, "
                    f"topic={args.path_topic}, frame={args.frame_id}",
                    flush=True)
                last_planned_start_pos = start_pos.copy()  # 更新上次规划位置
                last_planned_end_pos = end_pos.copy()  # 更新上次规划位置
            elif args.start_mode == 'current':
                current_plan_failed = True
                node.get_logger().error('PCT_PLAN_FAILED: no route from current start; no fixed-start fallback')
        except KeyboardInterrupt:
            print("路径发布失败")
        except Exception as exc:
            if args.start_mode != 'current':
                raise
            current_plan_failed = True
            node.get_logger().error('PCT_PLAN_FAILED: %s; navigation remains blocked' % exc)
    
    # 重新启动定时器（实现周期性检查）
    # Repeating ROS 2 timer is created once in pct_plan().


def processFeedback(feedback):
    """只负责更新end_pos，不直接执行规划"""
    global end_pos,start_pos
    p = feedback.pose.position
    # 更新目标位置
    if feedback.marker_name=="start_pos":
        if args.start_mode == 'current':
            return
        start_pos = np.array([p.x, p.y, p.z-0.5], dtype=np.float32)
    elif feedback.marker_name=="end_pos":
        end_pos = np.array([p.x, p.y, p.z-0.5], dtype=np.float32)

def normalizeQuaternion( quaternion_msg ):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s


def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x =  0.4
    marker.scale.y =  0.4
    marker.scale.z =  0.4
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def make6DofMarker(position,name,fixed=True,show_6dof = True):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.name = name
    int_marker.description = f"{name} 6-DOF Controller"
    int_marker.pose.position= Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
    makeBoxControl(int_marker)

    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        normalizeQuaternion(control.orientation)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
    server.insert(int_marker, feedback_callback=processFeedback)

    server.applyChanges()



def pct_plan():
    planner.loadTomogram(tomo_file)
    publish_map_clouds()
    global cloud_timer
    # The map clouds can precede the map-to-odom TF.  Send one live copy after
    # RViz and that TF are ready, without streaming large static clouds.
    cloud_timer = node.create_timer(0.5, publish_clouds_when_rviz_is_ready)

    if args.start_mode == 'fixed':
        make6DofMarker(start_pos,"start_pos", show_6dof=True)
    else:
        node.create_subscription(Odometry, args.start_pose_topic, on_current_start,
                                 qos_profile_sensor_data)
        node.get_logger().info('PCT_WAIT_CURRENT_START: waiting for accepted localization on %s' %
                               args.start_pose_topic)
    make6DofMarker(end_pos, "end_pos",show_6dof=True)
    
    # print("初始目标位置", end_pos)
    
    # 启动定时器（首次延迟PLAN_INTERVAL后执行）
    global plan_timer
    plan_timer = node.create_timer(PLAN_INTERVAL, plan_callback)
    return




if __name__ == '__main__':
    global server

    server = InteractiveMarkerServer(node, "basic_controls")
    pct_plan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
