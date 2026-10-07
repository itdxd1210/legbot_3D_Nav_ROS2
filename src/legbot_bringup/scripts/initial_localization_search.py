"""Map-derived height hypotheses; one shared BBS/NDT engine, bounded memory.

Search inputs are a gravity-level map and a standing scan. No spawn, route,
Gazebo pose, or known floor is used. Horizontal surfaces are hypotheses, not
floor labels: the complete 3-D scan must distinguish them in admission.
"""
from dataclasses import replace
import math
import time
import numpy as np
from scipy.ndimage import gaussian_filter1d, binary_dilation
from scipy.signal import find_peaks
from initial_localization_common import candidate_matrix, transform_points


def map_height_hypotheses(target):
    points = np.asarray(target)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 100:
        raise ValueError('Too few map points for height hypotheses')
    low, high = np.percentile(points[:, 2], [.5, 99.5])
    if high-low > 100:
        raise ValueError('Map height range exceeds bounded initial search')
    edges = np.arange(math.floor(low/.05)*.05-.1, high+.15, .05)
    counts, _ = np.histogram(points[:, 2], bins=edges)
    smooth = gaussian_filter1d(counts.astype(float), 1.)
    peaks, _ = find_peaks(smooth, distance=16,
                         prominence=max(80., .005*smooth.max()))
    if not len(peaks) or len(peaks) > 12:
        raise ValueError('No supported height bands or too many; do not guess a floor')
    return [(float(edges[i]+edges[i+1])/2) for i in peaks]


def geometry_cost(candidate, points, tree):
    """Capped full-scan squared distance, including unmatched points (m^2)."""
    sample = points[::max(1, len(points)//1500)]
    distances, _ = tree.query(transform_points(sample, candidate_matrix(candidate)), workers=1)
    return float(np.mean(np.minimum(distances, .7)**2)) if len(sample) else float('inf')


class MultiHeightSearch:
    def __init__(self, engine, target, tree, base_height):
        self.engine, self.target, self.tree = engine, target, tree
        self.base_height = base_height
        self.last_query_stats = {}
        self.heights = map_height_hypotheses(target)
        occupancy = engine.occupancy_map
        self.grids = []
        for height in self.heights:
            selected = target[(target[:, 2] >= height+.5) & (target[:, 2] <= height+2.5)]
            grid = np.zeros_like(occupancy.occupied)
            cells = np.floor((selected[:, :2]-[occupancy.origin_x_m, occupancy.origin_y_m])/
                             occupancy.resolution_m).astype(int)
            valid = ((cells[:, 0] >= 0) & (cells[:, 0] < grid.shape[1]) &
                     (cells[:, 1] >= 0) & (cells[:, 1] < grid.shape[0]))
            cells = cells[valid]
            grid[cells[:, 1], cells[:, 0]] = True
            # Same one-cell dilation as the existing engine preset.
            self.grids.append(binary_dilation(grid, structure=np.ones((3, 3))))
        self.normals = np.zeros_like(target, dtype=float)
        self.planar = np.zeros(len(target), dtype=bool)
        # Chunked neighborhoods avoid a large temporary allocation on 16 GB PCs.
        for start in range(0, len(target), 8000):
            _, indices = tree.query(target[start:start+8000], k=20, workers=1)
            neighbors = target[indices]
            centered = neighbors-neighbors.mean(axis=1, keepdims=True)
            values, vectors = np.linalg.eigh(np.einsum('nki,nkj->nij', centered, centered)/20)
            self.normals[start:start+8000] = vectors[:, :, 0]
            self.planar[start:start+8000] = (
                (values[:, 0] < .08*np.maximum(values.sum(axis=1), 1e-8)) &
                (values[:, 1] > .15*values[:, 2]))

    def refine(self, candidate, points):
        """Bounded point-to-plane ICP: XYZ/yaw only, preserving gravity leveling."""
        sample = points[::max(1, len(points)//4000)]
        position = np.array([candidate.x_m, candidate.y_m, candidate.z_m])
        origin = position.copy()
        yaw = candidate.yaw_rad
        for _ in range(25):
            co, si = math.cos(yaw), math.sin(yaw)
            transformed = sample @ np.array([[co, si, 0], [-si, co, 0], [0, 0, 1]])+position
            distances, indices = self.tree.query(transformed, workers=1)
            mask = (distances < .5) & self.planar[indices]
            if mask.sum() < 80:
                return candidate
            normals, source = self.normals[indices[mask]], sample[mask]
            residual = np.sum(normals*(transformed[mask]-self.target[indices[mask]]), axis=1)
            derivative = np.c_[-si*source[:, 0]-co*source[:, 1],
                                co*source[:, 0]-si*source[:, 1], np.zeros(len(source))]
            jacobian = np.c_[normals, np.sum(normals*derivative, axis=1)]
            weights = np.sqrt(np.minimum(1., .1/np.maximum(abs(residual), 1e-8)))
            delta = np.linalg.lstsq(jacobian*weights[:, None], -residual*weights, rcond=None)[0]
            if not np.isfinite(delta).all():
                return candidate
            delta[:3] *= min(1., .25/max(np.linalg.norm(delta[:3]), 1e-8))
            delta[3] = np.clip(delta[3], -.08, .08)
            position += delta[:3]
            yaw += delta[3]
            if np.linalg.norm(position-origin) > 1.5 or abs(yaw-candidate.yaw_rad) > math.radians(20):
                return candidate
            if np.linalg.norm(delta) < 1e-5:
                break
        return replace(candidate, x_m=float(position[0]), y_m=float(position[1]),
                       z_m=float(position[2]), yaw_rad=float(yaw))

    def query(self, points, progress_callback=None):
        engine = self.engine
        config, grid, scorer = engine.config, engine.matching_grid, engine.registration_scorer
        pool = []
        self.last_query_stats = {'bbs_candidates_per_height_limit': 256,
                                 'ndt_candidates_limit': 48, 'bbs': []}
        try:
            # Reuse the engine; do not duplicate the map/NDT target per height.
            engine.registration_scorer = None
            for index, (height, matching_grid) in enumerate(zip(self.heights, self.grids)):
                if progress_callback:
                    progress_callback('height_search', index, len(self.heights))
                engine.config = replace(config, seed_z_m=height+self.base_height,
                                        max_candidates=256, nms_radius_m=.5)
                engine.matching_grid = matching_grid
                started = time.monotonic()
                result = engine.query(points)
                top = max((c.bbs_score for c in result.candidates), default=0.)
                self.last_query_stats['bbs'].append({
                    'height_m': height, 'candidate_count': len(result.candidates),
                    'top_score': top,
                    'top_score_ties': sum(abs(c.bbs_score-top) < 1e-12 for c in result.candidates),
                    'wall_seconds': round(time.monotonic()-started, 3)})
                pool.extend(result.candidates)
        finally:
            engine.config, engine.matching_grid, engine.registration_scorer = config, grid, scorer
        pool.sort(key=lambda c: geometry_cost(c, points, self.tree))
        self.last_query_stats['raw_candidate_count'] = len(pool)
        self.last_query_stats['ndt_candidate_count'] = min(48, len(pool))
        candidates = engine._score_with_registration(points, pool[:48], progress_callback=progress_callback)
        candidates.sort(key=lambda c: geometry_cost(c, points, self.tree))
        refined = []
        for index, candidate in enumerate(candidates[:16]):
            if progress_callback:
                progress_callback('plane_refinement', index, min(16, len(candidates)))
            if candidate.registration_converged:
                refined.append(self.refine(candidate, points))
        return replace(result, candidates=refined, registration_scoring_enabled=True,
                       registration_scoring_backend='g2_ndt_score+plane_icp')
