"""Test current/fixed PCT start wiring and actual frontend callbacks without simulation."""
import ast
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

ROOT = Path(__file__).resolve().parents[1]
script = ROOT / 'src/pct_planner/planner/scripts/plan.py'
names = {'pct_start_from_odometry', 'check_current_start_terrain',
         'on_current_start', 'plan_callback', 'processFeedback'}
tree = ast.parse(script.read_text())
ns = dict(np=np)
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)
                             and n.name in names], type_ignores=[]), str(script), 'exec'), ns)

odom = Odometry()
odom.header.frame_id = 'initial_map'
odom.child_frame_id = 'base'
odom.header.stamp.sec = 10
odom.pose.pose.orientation.w = 1.
rotation = Rotation.from_euler('xyz', [.1, -.2, .7])
transform = [8., -4., 1.2, *rotation.as_quat()]
point = np.array([2., -3., .32])
mapped = rotation.apply(point) + transform[:3]
odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = mapped
assert np.allclose(ns['pct_start_from_odometry'](odom, transform, 'initial_map', 10.1), point)
for field, value in [('frame', 'odom'), ('child', 'livox_imu_link'), ('time', 11.),
                     ('time', 9.9), ('position', float('nan')), ('quaternion', 0.)]:
    bad = copy.deepcopy(odom)
    now = 10.1
    if field == 'frame': bad.header.frame_id = value
    if field == 'child': bad.child_frame_id = value
    if field == 'time': now = value
    if field == 'position': bad.pose.pose.position.x = value
    if field == 'quaternion': bad.pose.pose.orientation.w = value
    try:
        ns['pct_start_from_odometry'](bad, transform, 'initial_map', now)
    except ValueError:
        pass
    else:
        raise AssertionError('Invalid pose admitted: '+field)

logs, routes, starts = [], [], []
grid = np.ones((1, 3, 6))
planner = SimpleNamespace(map_dim=[3, 6], slice_dh=.5, layer_elev_grids=grid*.1,
                          tomogram=grid[None], match_best_layer=lambda *p: 0,
                          pos2idx=lambda p: np.array([4., 1.]))
planner.plan = lambda start, end, optimize: starts.append(start.copy()) or np.array([start, end])
node = SimpleNamespace(get_logger=lambda:SimpleNamespace(info=logs.append,error=logs.append),
                       get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10100000000,
                                                                                    to_msg=lambda:None)))
ns.update(args=SimpleNamespace(start_mode='current',map_from_pct=transform,
                              start_pose_frame='initial_map',optimize=False,
                              frame_id='building_pct',path_topic='/pct_path_raw'),
          node=node, planner=planner, start_pos=None, current_start_msg=None,
          current_plan_failed=False, end_pos=np.array([3.,-2.,1.]),
          last_planned_start_pos=None,last_planned_end_pos=None,plan_timer=None,
          make6DofMarker=lambda *a,**k:None, traj2ros=lambda *a:a[0],
          path_pub=SimpleNamespace(publish=routes.append))
ns['plan_callback']()
assert not routes and not starts  # No localization, no planning/default-start fallback.
ns['on_current_start'](odom)
ns['plan_callback']()
assert len(routes)==1 and np.allclose(starts[0],point)
other = copy.deepcopy(odom);other.pose.pose.position.x += 2.
ns['on_current_start'](other);ns['plan_callback']()
assert len(routes)==1 and np.allclose(ns['start_pos'],point)  # Freeze this mission's start.
for index in ([6.,1.], [4.,3.], [-1.,1.]):
    planner.pos2idx=lambda p,index=index:np.array(index)
    try: ns['check_current_start_terrain'](point,planner)
    except ValueError: pass
    else: raise AssertionError('Out-of-map point admitted')
planner.pos2idx=lambda p:np.array([4.,1.])
planner.layer_elev_grids[0,1,4]=-100.
ns.update(start_pos=None,current_plan_failed=False)
ns['plan_callback']()
assert ns['current_plan_failed'] and len(routes)==1 and ns['start_pos'] is None
planner.layer_elev_grids[0,1,4]=.1
planner.plan=lambda *a,**k:None
ns.update(start_pos=None,current_plan_failed=False,
          last_planned_start_pos=None,last_planned_end_pos=None)
ns['plan_callback']();ns['plan_callback']()
assert ns['current_plan_failed'] and len(routes)==1

def launch(name):
    path=ROOT/'src/legbot_bringup/launch'/name
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    return module,context

module,context=launch('pct_cross_floor_demo.launch.py')
# The source checkout does not include personal map data. Use a synthetic
# coordinate contract; the assertions below test wiring, not map quality.
original=dict(world_from_pct=[0.,0.,0.,0.,0.,0.,1.],
              map_from_pct=[0.,0.,0.,0.,0.,0.,1.],
              start=[0.,0.,.5],goal=[2.,0.,.5],spawn=[-6.,7.,.5],
              tomogram='test_map')
with tempfile.TemporaryDirectory() as folder:
    path=Path(folder)/'navigation.json'
    assets=Path(folder)/'initial_localization';assets.mkdir();(assets/'occupancy.yaml').write_text('fixture')
    context.launch_configurations.update(map_config=str(path),initial_localization='true',
        navigation_source='fastlio',align_with_ground_truth='false',localization_correction='false')
    for mode in ('fixed','current',None):
        config=dict(original)
        if mode is None: config.pop('start_mode',None)
        else: config['start_mode']=mode
        if mode=='current': config.pop('start')  # Current mode must not require a fixed start.
        path.write_text(json.dumps(config))
        actions=module.setup(context)
        arguments=next(dict(a.launch_arguments) for a in actions
                       if isinstance(a,IncludeLaunchDescription) and 'tomogram' in dict(a.launch_arguments))
        assert arguments['start_mode']==(mode or 'fixed')
        assert arguments['goal']==' '.join(map(str,original['goal']))
        if mode=='current':
            assert arguments['start']==''
            assert arguments['start_pose_topic']=='/initial_localization/odometry_base'
            assert arguments['map_from_pct']==' '.join(map(str,original['map_from_pct']))
        else: assert arguments['start']==' '.join(map(str,original['start']))
        child,child_context=launch('pct_plan.launch.py')
        child_context.launch_configurations.update(arguments)
        nodes=child.setup(child_context)
        cli=nodes[0]._Node__arguments
        assert cli  # Native planner node keeps existing route implementation.
    config=dict(original,start_mode='typo');path.write_text(json.dumps(config))
    try: module.setup(context)
    except ValueError as exc: assert 'start_mode' in str(exc)
    else: raise AssertionError('Unknown start mode accepted')
    config=dict(original,start_mode='current');path.write_text(json.dumps(config))
    context.launch_configurations.update(initial_localization='false',align_with_ground_truth='true')
    try: module.setup(context)
    except ValueError as exc: assert 'initial_localization' in str(exc)
    else: raise AssertionError('Current start without scan-only localization admitted')

print('PASS PCT start modes: SE(3), pose freshness/frames, terrain bounds, no fallback, one-shot start, launch compatibility')
