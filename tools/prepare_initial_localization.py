#!/usr/bin/python3
"""Prepare small global-search assets; preserve the original LIO/PCT map."""
import argparse
from pathlib import Path
import sys
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/legbot_bringup/scripts'))
from initial_localization_common import read_xyz_pcd, fit_ground, transform_points


def prepare(source, output):
    # No alignment.json, navigation route, world pose or truth is read here.
    points = read_xyz_pcd(source)
    level_from_map, ground = fit_ground(points)
    pieces = []
    for start in range(0, len(points), 200000):
        part = transform_points(points[start:start+200000], level_from_map)
        part = part[np.isfinite(part).all(axis=1)]
        _, indices = np.unique(np.floor(part/.20).astype(np.int64), axis=0, return_index=True)
        pieces.append(part[indices].astype(np.float32))
    reduced = np.concatenate(pieces)
    _, indices = np.unique(np.floor(reduced/.20).astype(np.int64), axis=0, return_index=True)
    reduced = reduced[indices]
    structure = reduced[(reduced[:, 2] >= .50) & (reduced[:, 2] <= 2.50)]
    if len(structure) < 100:
        raise ValueError('Not enough structural points above the ground')
    resolution = .25
    origin = np.floor(structure[:, :2].min(axis=0)/resolution)*resolution - resolution
    cells = np.floor((structure[:, :2]-origin)/resolution).astype(int)
    size = cells.max(axis=0)+2
    if np.prod(size) > 4000000:
        raise ValueError('Unexpected map extents; refuse oversized occupancy')
    image = np.full((size[1], size[0]), 255, dtype=np.uint8)
    image[cells[:, 1], cells[:, 0]] = 0
    output.mkdir(parents=True)
    with (output/'search_map.pcd').open('wb') as stream:
        stream.write(('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                      f'WIDTH {len(reduced)}\nHEIGHT 1\nPOINTS {len(reduced)}\nDATA binary\n').encode())
        stream.write(reduced.astype('<f4').tobytes())
    with (output/'occupancy.pgm').open('wb') as stream:
        stream.write(f'P5\n{size[0]} {size[1]}\n255\n'.encode())
        stream.write(np.flipud(image).tobytes())
    metadata = {'image': 'occupancy.pgm', 'resolution': resolution,
                'origin': [float(origin[0]), float(origin[1]), 0.], 'negate': 0,
                'occupied_thresh': .65, 'free_thresh': .196,
                'source_map': str(source.resolve()), 'source_size': source.stat().st_size,
                'source_mtime_ns': source.stat().st_mtime_ns,
                'level_from_map': level_from_map.tolist(), 'ground_fit': ground,
                'map_frame': 'initial_map', 'search_map': 'search_map.pcd',
                'base_height_m': .31, 'scan_range_m': 20.,
                'scope': 'ground-floor XY/yaw global search, gravity roll/pitch, approximate standing height'}
    (output/'occupancy.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    print(f'Prepared {len(reduced)} points; ground={ground}; assets={output}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_pcd', type=Path)
    args = parser.parse_args()
    output = args.map_pcd.resolve().parent/'initial_localization'
    if output.exists():
        raise SystemExit('Assets already exist; will not overwrite: ' + str(output))
    prepare(args.map_pcd, output)
