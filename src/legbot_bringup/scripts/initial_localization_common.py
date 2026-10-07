"""Frame math and bounded PCD loading for the initial-localization test."""
from pathlib import Path
from types import SimpleNamespace
import math
import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    result[:3, 3] = [p.x, p.y, p.z]
    return result


def transform_points(points, transform):
    return np.asarray(points) @ transform[:3, :3].T + transform[:3, 3]


def level_scan(points_odom, odom_from_base):
    # Registered scans are already deskewed and include the tilted LiDAR
    # extrinsics. Remove only the odometry translation and yaw, not gravity.
    yaw = math.atan2(odom_from_base[1, 0], odom_from_base[0, 0])
    level_from_odom = np.eye(4)
    level_from_odom[:3, :3] = Rotation.from_euler('z', -yaw).as_matrix()
    level_from_odom[:3, 3] = -level_from_odom[:3, :3] @ odom_from_base[:3, 3]
    return transform_points(points_odom, level_from_odom), level_from_odom @ odom_from_base


def candidate_matrix(candidate):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('z', candidate.yaw_rad).as_matrix()
    result[:3, 3] = [candidate.x_m, candidate.y_m, candidate.z_m]
    return result


def padded_bbs_search(search):
    """Keep coarse BBS bounds valid at map edges without changing map poses."""
    def query(grid, scan, resolution, angular_resolution, depth, count,
              nms_radius_cells=0):
        # Coarse block origins can put scan offsets one cell outside a pyramid
        # level although finer translations hit the original map. One coarsest
        # block of empty cells leaves room for the upper-bound dilation. The
        # multiple of 2**depth also preserves all original pyramid block splits.
        padding = 1 << depth
        candidates = search(np.pad(grid, padding, constant_values=False), scan,
                            resolution, angular_resolution, depth, count,
                            nms_radius_cells=nms_radius_cells)
        # The upstream C++ candidates are read-only. Restore grid indices here;
        # its normal cell-to-world conversion then retains the original origin.
        return [SimpleNamespace(tx_cell=c.tx_cell-padding,
                                ty_cell=c.ty_cell-padding,
                                yaw_index=c.yaw_index, yaw_rad=c.yaw_rad,
                                score=c.score, hit_count=c.hit_count,
                                point_count=c.point_count) for c in candidates]
    return query


def accepted_map_odom(result, odometry, expected_frame='initial_map'):
    """Use the scan timestamp, never latest odometry or a rejected best pose."""
    if result.get('decision') != 'candidate_passed_review':
        return None, result.get('decision', 'missing_decision')
    if result.get('frame') != expected_frame:
        return None, 'wrong_map_frame'
    try:
        stamp = float(result['stamp_s'])
        map_base = np.asarray(result['map_from_base'], dtype=float)
        if (not math.isfinite(stamp) or map_base.shape != (4, 4)
                or not np.isfinite(map_base).all()
                or not np.allclose(map_base[3], [0, 0, 0, 1])
                or not np.allclose(map_base[:3, :3].T @ map_base[:3, :3], np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(map_base[:3, :3]), 1., atol=1e-5)):
            return None, 'invalid_pose'
    except (KeyError, TypeError, ValueError):
        return None, 'invalid_pose'
    if not odometry:
        return None, 'waiting_synced_odom'
    time, odom_base = min(odometry, key=lambda item: abs(item[0]-stamp))
    if abs(time-stamp) > .05:
        return None, 'waiting_synced_odom'
    return map_base @ np.linalg.inv(odom_base), 'accepted'


def read_xyz_pcd(path):
    path = Path(path)
    with path.open('rb') as stream:
        header = {}
        for _ in range(50):
            line = stream.readline().decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split()
            header[key] = values
            if key == 'DATA':
                offset = stream.tell()
                break
        else:
            raise ValueError('PCD has no DATA header')
    schema = {'FIELDS': ['x', 'y', 'z'], 'SIZE': ['4']*3, 'TYPE': ['F']*3,
              'COUNT': ['1']*3, 'DATA': ['binary']}
    if any(header.get(k) != v for k, v in schema.items()):
        raise ValueError('Expected binary XYZ float32 PCD: ' + str(path))
    count = int(header['POINTS'][0])
    if count <= 0 or path.stat().st_size != offset + count*12:
        raise ValueError('Invalid PCD length')
    return np.memmap(path, mode='r', dtype='<f4', offset=offset, shape=(count, 3))


def fit_ground(points):
    """Fit the dominant low, approximately horizontal plane from the map only."""
    sample = np.asarray(points[::max(1, len(points)//200000)], dtype=float)
    sample = sample[np.isfinite(sample).all(axis=1)]
    low = sample[sample[:, 2] <= np.percentile(sample[:, 2], 55)]
    design = np.c_[low[:, :2], np.ones(len(low))]
    mask = np.ones(len(low), dtype=bool)
    for _ in range(8):
        a, b, c = np.linalg.lstsq(design[mask], low[mask, 2], rcond=None)[0]
        residual = low[:, 2] - design @ [a, b, c]
        mask = abs(residual - np.median(residual[mask])) < .08
        if mask.sum() < 100:
            raise ValueError('Insufficient ground-plane support')
    normal = np.array([-a, -b, 1.])
    normal /= np.linalg.norm(normal)
    angle = math.degrees(math.acos(float(normal[2])))
    support = float(np.mean(abs(sample @ normal - c*normal[2]) < .08))
    if angle > 5 or support < .25:
        raise ValueError('No dominant outdoor ground plane; do not guess a floor')
    axis = np.cross(normal, [0., 0., 1.])
    length = np.linalg.norm(axis)
    level_from_map = np.eye(4)
    if length > 1e-10:
        level_from_map[:3, :3] = Rotation.from_rotvec(
            axis/length * math.radians(angle)).as_matrix()
    level_from_map[2, 3] = -c*normal[2]
    return level_from_map, {'tilt_deg': angle, 'support_fraction': support,
                            'ground_z_at_origin_m': float(c)}


def candidate_overlap(candidate, points, tree, base_height):
    """Score structure separately so a large floor cannot hide wrong matches."""
    distances, _ = tree.query(transform_points(points, candidate_matrix(candidate)), workers=1)
    structure = (points[:, 2] >= .50-base_height) & (points[:, 2] <= 2.50-base_height)
    def metrics(values):
        inside = values < .30
        ratio = float(inside.mean()) if len(values) else 0.
        rmse = float(np.sqrt(np.mean(values[inside]**2))) if inside.any() else float('inf')
        return ratio, rmse
    ratio, rmse = metrics(distances)
    structure_ratio, structure_rmse = metrics(distances[structure])
    return ratio, rmse, structure_ratio, structure_rmse, int(structure.sum())


def review_candidates(candidates, points, tree, base_height, height_hypotheses=None):
    """Initial-pose admission; never uses truth or a known XY/yaw."""
    structure = (points[:, 2] >= .50-base_height) & (points[:, 2] <= 2.50-base_height)
    if structure.sum() < 80:
        return None, 'insufficient_structure', []
    reviews = []
    for candidate in candidates:
        if (not candidate.registration_converged or candidate.registration_fitness is None
                or not math.isfinite(candidate.registration_fitness)
                or not all(math.isfinite(v) for v in (candidate.x_m, candidate.y_m,
                                                       candidate.z_m, candidate.yaw_rad))):
            continue
        heights = [0.] if height_hypotheses is None else height_hypotheses
        if not heights or min(abs(candidate.z_m-h-base_height) for h in heights) > .5:
            continue
        ratio, rmse, sr, se, _ = candidate_overlap(candidate, points, tree, base_height)
        reviews.append((candidate, ratio, rmse, sr, se))
    if height_hypotheses is None:
        reviews.sort(key=lambda item: (-item[3], -item[1], item[4], item[2]))
    else:
        from initial_localization_search import geometry_cost
        reviews.sort(key=lambda item: geometry_cost(item[0], points, tree))
    if not reviews:
        return None, 'no_converged_height_candidate', []
    best, ratio, rmse, structure_ratio, structure_rmse = reviews[0]
    # Preserve the public review tuple used by diagnostics and callers.
    public_reviews = [item[:3] for item in reviews]
    if ratio < .60 or rmse > .20 or structure_ratio < .85 or structure_rmse > .20:
        return best, 'insufficient_overlap', public_reviews
    for other, other_ratio, other_rmse, other_sr, other_se in reviews[1:]:
        separation = math.hypot(other.x_m-best.x_m, other.y_m-best.y_m)
        yaw_delta = abs(math.atan2(math.sin(other.yaw_rad-best.yaw_rad), math.cos(other.yaw_rad-best.yaw_rad)))
        similar = (other_ratio >= ratio-.05 and other_rmse <= rmse+.03
                   and other_sr >= structure_ratio-.05 and other_se <= structure_rmse+.03)
        if height_hypotheses is not None:
            similar = (geometry_cost(other, points, tree) <=
                       1.20*geometry_cost(best, points, tree)+.0005)
        if ((separation > .5 or abs(other.z_m-best.z_m) > .5 or yaw_delta > math.radians(10))
                and similar):
            return best, 'ambiguous_candidates', public_reviews
    return best, 'candidate_passed_review', public_reviews
