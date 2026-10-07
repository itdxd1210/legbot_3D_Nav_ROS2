import ast
import importlib.util
import json
import runpy
import sys
import tempfile
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rclpy
from rclpy.clock import Clock
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

root=Path(__file__).resolve().parents[1]
scripts=root/'src/legbot_bringup/scripts'
sys.path.insert(0,str(scripts))
from initial_localization_common import accepted_map_odom
helpers=runpy.run_path(str(scripts/'initial_localization_bridge'))
Bridge=helpers['InitialBridge']
clock=Clock()
stamp=clock.now().to_msg()
seconds=stamp.sec+stamp.nanosec*1e-9
odom=Odometry()
odom.header.stamp=stamp
odom.header.frame_id='odom'
odom.child_frame_id='base'
odom.pose.pose.orientation.w=1.
odom.twist.twist.linear.x=.4
pose=np.eye(4);pose[:3,3]=[4.,-2.,.1]
result=dict(decision='candidate_passed_review',frame='initial_map',stamp_s=seconds,map_from_base=pose.tolist())
assert accepted_map_odom(dict(result,decision='ambiguous_candidates'),[(seconds,np.eye(4))])[0] is None
assert accepted_map_odom(result,[(seconds+.1,np.eye(4))])[0] is None
assert accepted_map_odom(dict(result,frame='wrong'),[(seconds,np.eye(4))])[0] is None
outputs=[];seeds=[];logs=[]
node=SimpleNamespace(map_frame='initial_map',map_odom=None,result=None,seeded=False,
    history=deque(),get_clock=lambda:clock,
    get_logger=lambda:SimpleNamespace(info=logs.append,warn=logs.append),
    pose_pub=SimpleNamespace(publish=outputs.append),
    seed_pub=SimpleNamespace(publish=seeds.append,get_subscription_count=lambda:1))
node.admit=lambda:Bridge.admit(node)
Bridge.on_odom(node,odom)
assert not outputs and not seeds
Bridge.on_result(node,String(data=json.dumps(dict(result,decision='ambiguous_candidates'))))
assert node.map_odom is None
Bridge.on_result(node,String(data=json.dumps(result)))
Bridge.on_odom(node,odom)
assert len(outputs)==len(seeds)==1 and outputs[0].header.frame_id=='initial_map'
assert outputs[0].pose.pose.position.x==4. and outputs[0].twist.twist.linear.x==.4
assert seeds[0].header.frame_id=='localization_probe'
Bridge.on_odom(node,odom)
assert len(outputs)==2 and len(seeds)==1

path=root/'src/legbot_bringup/launch/pct_cross_floor_demo.launch.py'
spec=importlib.util.spec_from_file_location('demo',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
context=LaunchContext()
for action in module.generate_launch_description().entities:
    if isinstance(action,DeclareLaunchArgument):action.execute(context)
with tempfile.TemporaryDirectory() as folder:
    fixture=Path(folder)
    (fixture/'initial_localization').mkdir()
    (fixture/'initial_localization/occupancy.yaml').write_text('fixture')
    (fixture/'map_odom.pcd').write_text('fixture')
    (fixture/'alignment.json').write_text('{}')
    config=fixture/'navigation.json'
    config.write_text(json.dumps(dict(world_from_pct=[0.,0.,0.,0.,0.,0.,1.],
        map_from_pct=[0.,0.,0.,0.,0.,0.,1.],start=[0.,0.,.5],
        goal=[2.,0.,.5],spawn=[-6.,7.,.5],tomogram='test_map')))
    context.launch_configurations.update(navigation_source='fastlio',initial_localization='true',
        align_with_ground_truth='false',map_config=str(config),localization_correction='true')
    actions=module.setup(context)
params={str(a.node_executable):evaluate_parameters(context,a._Node__parameters)[0] for a in actions if isinstance(a,Node)}
assert params['pct_path_mission']['map_pose_topic']=='/initial_localization/odometry_base'
assert params['pct_path_mission']['world_from_pct']==params['localization_navigation_bridge']['map_from_pct']
assert params['localization_comparison']['seed_initial_pose'] is False
assert params['initial_localization_bridge']['seed_map_matching'] is True
tree=ast.parse((scripts/'initial_localization_bridge').read_text())
topics=[c.args[1].value for c in ast.walk(tree) if isinstance(c,ast.Call) and isinstance(c.func,ast.Attribute)
        and c.func.attr=='create_subscription' and len(c.args)>1 and isinstance(c.args[1],ast.Constant)]
assert set(topics)=={'/fast_lio/odometry_base','/initial_localization/result'}
nav_helpers=runpy.run_path(str(scripts/'localization_navigation_bridge'))
rclpy.init(args=['--ros-args','-p','map_from_pct:=[1.0,2.0,3.0,0.0,0.0,0.0,1.0]'])
native_bridge=nav_helpers['NavigationBridge']()
assert np.allclose(native_bridge.pct_map[:3,3],[-1.,-2.,-3.])
native_bridge.destroy_node()
rclpy.shutdown()
print('PASS admission gating, exact-time anchoring, unchanged body twist, one-shot matching seed, no truth subscriptions, launch wiring')
