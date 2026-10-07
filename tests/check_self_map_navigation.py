"""Validate custom-map launch wiring without starting simulation or control."""
import importlib.util
import json
import math
from pathlib import Path
import runpy
import tempfile

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[1]


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'src/legbot_bringup/launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return module, context


module, context = load_launch('pct_cross_floor_demo.launch.py')
with tempfile.TemporaryDirectory() as folder:
    filename = Path(folder) / 'map.json'
    pose = [1., 2., 3., 0., math.sin(.15), 0., math.cos(.15)]
    filename.write_text(json.dumps(dict(world_from_pct=pose, start=[0., 0., .5],
        goal=[2., 0., .5], spawn=[-6., 7., .5], tomogram='test_map')))
    context.launch_configurations.update(map_config=str(filename), navigation_source='fastlio',
        align_with_ground_truth='true', localization_probe='false')
    actions = module.setup(context)
    bridge = next(a for a in actions if isinstance(a, Node) and str(a.node_executable) == 'pct_path_mission')
    params = evaluate_parameters(context, bridge._Node__parameters)[0]
    assert list(params['world_from_pct']) == pose
    assert params['spawn_x'] == -6. and params['spawn_y'] == 7.
    assert params['map_pose_topic'] == '/Odometry_gazebo'
    # A coordinate-preserving localization crop must not replace the PCT route
    # or change which alignment is used by the correction bridge.
    (Path(folder) / 'alignment.json').write_text('{}')
    full_pcd = Path(folder) / 'map_odom.pcd'
    full_pcd.write_text('fixture')
    crop_pcd = Path(folder) / 'indoor_crop.pcd'
    crop_pcd.write_text('fixture')
    context.launch_configurations.update(localization_correction='true')

    def matching_arguments(actions):
        includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
        return next(dict(a.launch_arguments) for a in includes
                    if 'scan_max_range' in dict(a.launch_arguments))

    assert matching_arguments(module.setup(context))['map_path'] == str(full_pcd)
    context.launch_configurations['localization_map_path'] = str(crop_pcd)
    actions = module.setup(context)
    assert matching_arguments(actions)['map_path'] == str(crop_pcd)
    correction_bridge = next(a for a in actions if isinstance(a, Node)
                             and str(a.node_executable) == 'localization_navigation_bridge')
    correction_params = evaluate_parameters(context, correction_bridge._Node__parameters)[0]
    assert correction_params['alignment_file'] == str(Path(folder) / 'alignment.json')
    assert correction_params['map_config'] == str(filename)
    mission = next(a for a in actions if isinstance(a, Node)
                   and str(a.node_executable) == 'pct_path_mission')
    mission_params = evaluate_parameters(context, mission._Node__parameters)[0]
    assert list(mission_params['world_from_pct']) == pose
    assert mission_params['odom_topic'] == '/navigation/odometry_base'
    assert mission_params['frame_id'] == 'nav_map'
    pct_args = next(dict(a.launch_arguments) for a in actions
                    if isinstance(a, IncludeLaunchDescription)
                    and 'tomogram' in dict(a.launch_arguments))
    assert pct_args['tomogram'] == 'test_map'
    full_pcd.unlink()
    assert matching_arguments(module.setup(context))['map_path'] == str(crop_pcd)
    context.launch_configurations['localization_map_path'] = str(Path(folder) / 'missing.pcd')
    try:
        module.setup(context)
    except ValueError as error:
        assert 'Localization PCD does not exist' in str(error)
    else:
        raise AssertionError('Missing localization PCD must be rejected')
    context.launch_configurations['localization_map_path'] = str(crop_pcd)
    context.launch_configurations['align_with_ground_truth'] = 'false'
    try:
        module.setup(context)
    except ValueError:
        pass
    else:
        raise AssertionError('Uncalibrated custom-map startup must be rejected')

helpers = runpy.run_path(str(ROOT / 'src/legbot_bringup/scripts/pct_path_mission'))
point = (4., -2., .7)
world_point = helpers['transform_xyz'](point, pose)
back = helpers['transform_xyz'](world_point, helpers['inverse_pose'](pose))
assert max(abs(a-b) for a,b in zip(point,back)) < 1e-9
print('PASS custom-map launch, localization-only crop override, missing PCD guard, unchanged PCT/alignment, full 3D transform round trip')
