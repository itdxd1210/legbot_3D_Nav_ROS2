"""Synthetic checks in an isolated ROS domain; never drives the simulator."""
import json
import runpy
import tempfile
import time
from pathlib import Path
import numpy as np
import rclpy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock

root = Path(__file__).resolve().parents[0]
scripts = Path('src/legbot_bringup/scripts')
Recorder = runpy.run_path(str(scripts/'mapping_pcd_recorder'))['Recorder']
Keyboard = runpy.run_path(str(scripts/'go2_mapping_keyboard'))['Keyboard']
with tempfile.TemporaryDirectory(prefix='mapping-unit-') as output:
    rclpy.init(args=['--ros-args', '-p', 'output_root:='+output,
                    '-p', 'chunk_points:=3', '-p', 'sample_period:=0.5'])
    node = Recorder()
    def cloud(stamp, points, frame='odom'):
        h=Header(); h.frame_id=frame; h.stamp.sec=stamp
        return point_cloud2.create_cloud_xyz32(h, points)
    points=np.arange(21,dtype=np.float32).reshape(-1,3)
    node.receive(cloud(1,points))
    assert node.count == 1 and len(node.chunks)==2 and not node.failed
    node.receive(cloud(1,points))
    assert node.count==1  # sampling suppresses duplicate stamps
    result=node.save_request(Trigger.Request(), Trigger.Response())
    assert result.success and node.count==0
    manifest=json.loads((node.directory/'manifest.json').read_text())
    assert [x['points'] for x in manifest['chunks']]==[3,3,1]
    loaded=[]
    for chunk in manifest['chunks']:
        content=(node.directory/chunk['file']).read_bytes().split(b'DATA binary\n')[1]
        loaded.append(np.frombuffer(content,dtype='<f4').reshape(-1,3))
    np.testing.assert_array_equal(np.concatenate(loaded),points)
    node.receive(cloud(2,[[1,2,float('nan')],[1,2,3]]))
    assert node.count==1
    node.reserve=10**18
    response=node.save_request(Trigger.Request(),Trigger.Response())
    assert not response.success and node.count==1 and node.failed
    node.reserve=0;node.flush();node.destroy_node()
    keyboard=Keyboard()
    class Publisher:
        def __init__(self):self.messages=[]
        def publish(self,msg):self.messages.append(msg)
    keyboard.pub=Publisher();keyboard.mode=Publisher();keyboard.start=Publisher()
    keyboard.key('w');keyboard.tick()
    assert keyboard.pub.messages[-1].linear.x==0  # no simulation clock
    keyboard.clock_received(Clock());keyboard.key('w');keyboard.tick()
    assert keyboard.pub.messages[-1].linear.x==.25
    keyboard.key('f');keyboard.tick()
    assert keyboard.pub.messages[-1].linear.x==0
    keyboard.key('w');keyboard.tick()
    assert keyboard.pub.messages[-1].linear.x==.4
    keyboard.key('h');keyboard.tick()
    assert keyboard.pub.messages[-1].linear.x==0  # switching gears stops motion
    keyboard.key('w');keyboard.tick()
    assert keyboard.pub.messages[-1].linear.x==.6
    keyboard.key('a');keyboard.tick()
    assert keyboard.pub.messages[-1].linear.y==.15  # high gear only changes forward speed
    keyboard.key('s');keyboard.tick()
    assert keyboard.pub.messages[-1].linear.x==-.15
    keyboard.key('g');keyboard.tick()
    assert keyboard.pub.messages[-1].linear.x==0
    keyboard.key('w');keyboard.tick()
    assert keyboard.pub.messages[-1].linear.x==.25
    keyboard.until=time.monotonic()-1;keyboard.tick()
    assert keyboard.pub.messages[-1].linear.x==0
    keyboard.key('q');keyboard.tick()
    assert keyboard.pub.messages[-1].angular.z==.35
    keyboard.key(' ');keyboard.tick()
    assert keyboard.pub.messages[-1].angular.z==0
    keyboard.key('b');assert keyboard.start.messages[-1].data
    keyboard.key('2');keyboard.tick();assert keyboard.mode.messages[-1].data==2
    keyboard.destroy_node();rclpy.shutdown()
print('PASS: chunk boundaries, exact XYZ roundtrip, tail flush, sample throttle, NaN filtering, disk guard, three forward gears, keyboard deadman/modes')
