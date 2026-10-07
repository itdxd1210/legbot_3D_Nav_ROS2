"""Offline numerical and ROS late-subscriber tests; run in an isolated ROS domain."""
import runpy
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

m=runpy.run_path('src/legbot_bringup/scripts/mapping_map_viewer')
v=m['VoxelPreview'](.1, 10)
v.add(np.array([[i,0,0] for i in range(100)] + [[float('nan'),0,0]],dtype=np.float32))
assert len(v.points)<=10 and v.voxel>.1
assert v.points[:,0].max()>80  # coarsen, do not truncate far end
v.add(v.points.copy());assert len(v.points)<=10
rclpy.init()
node=m['Viewer']()
h=Header();h.frame_id='odom';h.stamp.sec=1
node.receive(point_cloud2.create_cloud_xyz32(h, np.array([[1,2,0],[1,2,1],[1,2,2]],dtype=np.float32)))
node.publish()
listener=rclpy.create_node('preview_test_listener')
received=[]
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
sub=listener.create_subscription(PointCloud2,'/mapping/map_preview',received.append,q)
end=time.monotonic()+4
while not received and time.monotonic()<end:
    rclpy.spin_once(listener,timeout_sec=.05)
assert received and received[-1].width==3, 'Late subscriber missed snapshot'
node.set_parameters([rclpy.parameter.Parameter('z_min',value=.5),rclpy.parameter.Parameter('z_max',value=1.5)])
node.publish();received.clear()
end=time.monotonic()+3
while not received and time.monotonic()<end:rclpy.spin_once(listener,timeout_sec=.05)
assert received and received[-1].width==1
listener.destroy_node();node.destroy_node();rclpy.shutdown()
print('PASS: adaptive point cap, retained spatial extent, duplicate voxels, late subscription, Z filtering')
