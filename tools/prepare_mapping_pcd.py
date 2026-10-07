#!/usr/bin/env python3
"""Merge recorder chunks into a voxel-downsampled PCD; never alter source chunks."""
import argparse
import json
from pathlib import Path
import runpy

import open3d as o3d


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_directory', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--voxel', type=float, default=.03)
    parser.add_argument('--max-points', type=int, default=8000000)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new filename')
    root = Path(__file__).resolve().parents[1]
    helpers = runpy.run_path(str(root / 'src/legbot_bringup/scripts/mapping_map_viewer'))
    directory = args.map_directory.resolve()
    manifest = json.loads((directory / 'manifest.json').read_text())
    if not manifest.get('frame_id'):
        parser.error('Manifest has no frame_id')
    preview = helpers['VoxelPreview'](args.voxel, args.max_points)
    for chunk in manifest['chunks']:
        path = (directory / chunk['file']).resolve()
        if path.parent != directory:
            parser.error('Chunk path leaves map directory')
        for points in helpers['read_chunk'](path):
            preview.add(points)
    if not len(preview.points):
        parser.error('Map contains no finite points')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(preview.points))
    if not o3d.io.write_point_cloud(str(args.output), cloud):
        raise RuntimeError('PCD write failed')
    print('Saved %d points, voxel %.3f m, frame %s; original chunks unchanged' %
          (len(preview.points), preview.voxel, manifest['frame_id']))


if __name__ == '__main__':
    main()
