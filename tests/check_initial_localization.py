"""Offline frame, ambiguity and truth-isolation checks; no simulator."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

root = Path(__file__).resolve().parents[1]
scripts = root/'src/legbot_bringup/scripts'
sys.path.insert(0, str(scripts))
from initial_localization_common import (
    transform_points, level_scan, fit_ground, review_candidates, candidate_matrix,
    padded_bbs_search, candidate_overlap)
from initial_localization_search import map_height_hypotheses, MultiHeightSearch, geometry_cost

# Empty padding must leave map occupancy, world coordinates and scores intact.
grid = np.zeros((9, 13), dtype=bool)
grid[0, 2] = True
original_grid = grid.copy()
scan_xy = np.array([[-1., 2.]])
def checked_search(padded, scan, res, angle, depth, count, nms_radius_cells=0):
    padding = 1 << depth
    assert padded.shape == (9+2*padding, 13+2*padding)
    assert np.array_equal(padded[padding:-padding, padding:-padding], grid)
    assert padded.sum() == grid.sum()
    assert scan is scan_xy and (res, angle, depth, count, nms_radius_cells) == (.25, .1, 5, 24, 4)
    return [SimpleNamespace(tx_cell=padding+3, ty_cell=padding+6,
                            yaw_index=2, yaw_rad=.2, score=.9,
                            hit_count=9, point_count=10)]
edge = padded_bbs_search(checked_search)(grid, scan_xy, .25, .1, 5, 24,
                                       nms_radius_cells=4)[0]
assert (edge.tx_cell, edge.ty_cell) == (3, 6)
assert (edge.yaw_index, edge.yaw_rad, edge.score, edge.hit_count, edge.point_count) == (2, .2, .9, 9, 10)
assert np.array_equal(grid, original_grid)

# Exercise the actual compiled backend when the ROS overlay is sourced. A
# compact edge fixture previously lost a perfect pose to an interior distractor.
try:
    from ament_index_python.packages import get_package_prefix
    sys.path.insert(0, get_package_prefix('lidar_localization_ros2')+'/lib/lidar_localization_ros2')
    import bbs_cpp
except ImportError:
    print('SKIP compiled BBS edge regression: source the ROS overlay to enable it')
else:
    rng = np.random.default_rng(1)
    edge_grid = np.zeros((64, 128), dtype=np.uint8)
    edge_scan = np.c_[rng.integers(-24, -8, 24), rng.integers(-8, 8, 24)].astype(float)
    offsets = np.floor(.5+edge_scan).astype(int)
    edge_grid[32+offsets[:, 1], 25+offsets[:, 0]] = 1
    edge_grid[32+offsets[:19, 1], 85+offsets[:19, 0]] = 1
    raw = bbs_cpp.branch_and_bound_candidates(edge_grid, edge_scan, 1., np.pi/2, 5, 4, 4)
    exhaustive = bbs_cpp.branch_and_bound_candidates(edge_grid, edge_scan, 1., np.pi/2, 0, 4, 4)
    fixed = padded_bbs_search(bbs_cpp.branch_and_bound_candidates)(
        edge_grid, edge_scan, 1., np.pi/2, 5, 4, nms_radius_cells=4)
    assert raw[0].score < 1. and exhaustive[0].score == fixed[0].score == 1.
    assert (fixed[0].tx_cell, fixed[0].ty_cell) == (25, 32)
    assert abs(fixed[0].yaw_rad) < 1e-10
    print('PASS compiled BBS edge regression: padded search recovers exhaustive optimum')

# Arbitrary translated/yawed navigation odom; body pitch is not forced to zero.
odom = np.eye(4)
odom[:3, :3] = Rotation.from_euler('xyz', [.07, -.15, 1.3]).as_matrix()
odom[:3, 3] = [4., -7., .6]
base_scan = np.array([[2., 3., .1], [-1., 4., 2.], [6., -2., 1.]])
registered = transform_points(base_scan, odom)
level, level_from_base = level_scan(registered, odom)
assert np.allclose(level, transform_points(base_scan, level_from_base))
assert abs(np.arctan2(level_from_base[1, 0], level_from_base[0, 0])) < 1e-10
map_from_level = np.eye(4)
map_from_level[:3, :3] = Rotation.from_euler('xyz', [-.03, .04, .2]).as_matrix()
map_from_level[:3, 3] = [8., 2., -.4]
candidate = SimpleNamespace(x_m=12., y_m=3., z_m=.31, yaw_rad=-.9,
                            registration_converged=True, registration_fitness=.01)
map_from_base = map_from_level @ candidate_matrix(candidate) @ level_from_base
assert np.allclose(transform_points(base_scan, map_from_base),
                   transform_points(level, map_from_level @ candidate_matrix(candidate)))

# Ground fit uses only PCD points, recovers tilted/offset synthetic ground.
x, y = np.meshgrid(np.linspace(-20, 20, 80), np.linspace(-15, 15, 80))
floor = np.c_[x.ravel(), y.ravel(), (.01*x-.004*y-.45).ravel()]
stack = np.concatenate((floor, floor+[0, 0, 2]))
level_map, stats = fit_ground(stack)
assert np.max(np.abs(transform_points(floor, level_map)[:, 2])) < 1e-8
assert stats['support_fraction'] > .49

review_scan = np.random.default_rng(23).uniform([1., 2., .3], [4., 5., 1.9], (120, 3))
tree = cKDTree(transform_points(review_scan, candidate_matrix(candidate)))
best, reason, _ = review_candidates([candidate], review_scan, tree, .31)
assert best is candidate and reason == 'candidate_passed_review'
wrong = SimpleNamespace(**{**vars(candidate), 'x_m': 30.})
assert review_candidates([wrong], review_scan, tree, .31)[1] == 'insufficient_overlap'
alias_tree = cKDTree(np.vstack((tree.data, transform_points(review_scan, candidate_matrix(wrong)))))
assert review_candidates([candidate, wrong], review_scan, alias_tree, .31)[1] == 'ambiguous_candidates'
wrong.z_m = 3.31
assert review_candidates([wrong], review_scan, tree, .31)[0] is None
wrong.registration_fitness = None
assert review_candidates([wrong], review_scan, tree, .31)[0] is None
assert review_candidates([candidate], review_scan[:30], tree, .31)[1] == 'insufficient_structure'

# Heights are extracted from map surfaces, not simulator floor/spawn settings.
levels = np.vstack([np.c_[x.ravel(), y.ravel(), np.full(x.size, height)]
                    for height in (0., 1.5, 3., 4.5)])
heights = map_height_hypotheses(levels)
assert len(heights) == 4 and np.allclose(heights, [0., 1.5, 3., 4.5], atol=.06)
try:
    map_height_hypotheses(np.zeros((10, 3)))
except ValueError:
    pass
else:
    raise AssertionError('Insufficient map must not fabricate a floor')
upstairs = SimpleNamespace(**{**vars(candidate), 'z_m': 3.31})
upstairs_tree = cKDTree(transform_points(review_scan, candidate_matrix(upstairs)))
assert review_candidates([upstairs], review_scan, upstairs_tree, .31, heights)[1] == 'candidate_passed_review'
repeated_floor = cKDTree(np.vstack((tree.data, upstairs_tree.data)))
assert review_candidates([candidate, upstairs], review_scan, repeated_floor, .31, heights)[1] == 'ambiguous_candidates'

# Capped distance includes unmatched points; unlike inlier-only RMSE it cannot
# report a perfect score by dropping most of a scan.
assert geometry_cost(candidate, review_scan, tree) < 1e-20
assert geometry_cost(candidate, review_scan+[100, 0, 0], tree) > .48

# Point-to-plane refinement preserves roll/pitch and recovers a local XYZ/yaw
# offset. Exercise actual helper with three independent planar surfaces.
u, v = np.meshgrid(np.linspace(-3, 3, 35), np.linspace(0, 3, 30))
planes = np.vstack((np.c_[u.ravel(), v.ravel(), np.zeros(u.size)],
                    np.c_[np.zeros(u.size), u.ravel(), v.ravel()],
                    np.c_[u.ravel(), np.full(u.size, 3.), v.ravel()]))
class FixtureMap:
    occupied = np.zeros((40, 40), dtype=bool)
    origin_x_m = origin_y_m = -5.
    resolution_m = .25
fixture_engine = SimpleNamespace(occupancy_map=FixtureMap())
fixture_search = MultiHeightSearch(fixture_engine, planes, cKDTree(planes), .31)
from dataclasses import dataclass
@dataclass(frozen=True)
class FixtureCandidate:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
refined = fixture_search.refine(FixtureCandidate(.15, -.1, .1, .04), planes)
assert np.linalg.norm([refined.x_m, refined.y_m, refined.z_m]) < .02
assert abs(refined.yaw_rad) < .01

# A floor-dominated wrong location must not pass just because total overlap is high.
floor = np.c_[np.random.default_rng(9).uniform(-5., 5., (1000, 2)), np.full(1000, -.31)]
floor_scan = np.vstack((floor, review_scan))
floor_alias = SimpleNamespace(**{**vars(candidate), 'x_m': 30.})
floor_tree = cKDTree(np.vstack((transform_points(floor, candidate_matrix(floor_alias)),
                                transform_points(review_scan, candidate_matrix(candidate)))))
all_ratio, _, structure_ratio, _, count = candidate_overlap(floor_alias, floor_scan, floor_tree, .31)
assert all_ratio > .85 and structure_ratio == 0. and count == 120
assert review_candidates([floor_alias], floor_scan, floor_tree, .31)[1] == 'insufficient_overlap'

# Verify the actual input/output contracts, not just comments.
probe = ast.parse((scripts/'initial_localization_probe').read_text())
def literal_topic_calls(tree, method):
    return [call.args[1].value for call in ast.walk(tree)
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
            and call.func.attr == method and len(call.args) > 1
            and isinstance(call.args[1], ast.Constant)]
assert set(literal_topic_calls(probe, 'create_subscription')) == {
    '/fast_lio/odometry_base', '/fast_lio/cloud_registered', '/go2/demo_ready'}
assert '/initialpose' not in literal_topic_calls(probe, 'create_publisher')
observer = ast.parse((scripts/'initial_localization_truth').read_text())
assert set(literal_topic_calls(observer, 'create_subscription')) == {
    '/Odometry_gazebo', '/initial_localization/result'}
assert set(literal_topic_calls(observer, 'create_publisher')) == {'~/result', '~/pose'}
launch = (root/'src/legbot_bringup/launch/initial_localization_test.launch.py').read_text()
assert 'pct_plan.launch.py' not in launch and 'scan.launch.py' not in launch
assert '/initial_localization/unused_bspline' in launch
for path in [scripts/'initial_localization_probe', scripts/'initial_localization_truth',
             scripts/'initial_localization_search.py',
             root/'tools/prepare_initial_localization.py']:
    compile(path.read_text(), str(path), 'exec')
print('PASS initial localization: multi-height search, local refinement, cross-floor ambiguity, frame/truth isolation')
