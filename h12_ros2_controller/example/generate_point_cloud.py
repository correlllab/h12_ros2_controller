import argparse
import numpy as np

import os
import sys
sys.path.append(os.path.abspath(os.path.join(__file__, '../../..')))
from h12_ros2_controller.utility.path_definition import DUMMY_POINTCLOUD_PATH


def make_cube(center, size, samples):
    '''Sample a solid axis-aligned cube point cloud'''
    half = size / 2.0
    grid = np.linspace(-half, half, samples)
    x, y, z = np.meshgrid(grid, grid, grid, indexing='ij')
    points = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=-1)
    return points + np.asarray(center, dtype=float)


def make_sphere(center, size, samples):
    '''Sample a solid sphere point cloud of the given diameter'''
    radius = size / 2.0
    grid = np.linspace(-radius, radius, samples)
    x, y, z = np.meshgrid(grid, grid, grid, indexing='ij')
    points = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=-1)
    # keep only the samples inside the sphere
    points = points[np.linalg.norm(points, axis=1) <= radius]
    return points + np.asarray(center, dtype=float)


def main(shape, center, size, samples, output):
    '''Generate a dummy obstacle point cloud and save it to disk'''
    if shape == 'cube':
        points = make_cube(center, size, samples)
    else:
        points = make_sphere(center, size, samples)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    np.save(output, points.astype(np.float32))
    print(f'Saved {len(points)} {shape} points to {output}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Generate a dummy obstacle point cloud in the pelvis frame.',
    )
    parser.add_argument('--shape', choices=('cube', 'sphere'), default='sphere')
    parser.add_argument(
        '--center', type=float, nargs=3, default=(0.45, 0.30, 0.30),
        help='obstacle center in the pelvis frame [m]',
    )
    parser.add_argument(
        '--size', type=float, default=0.16,
        help='cube edge length or sphere diameter [m]',
    )
    parser.add_argument(
        '--samples', type=int, default=15,
        help='number of samples per axis',
    )
    parser.add_argument('--output', default=DUMMY_POINTCLOUD_PATH)
    args = parser.parse_args()
    main(
        shape=args.shape,
        center=tuple(args.center),
        size=args.size,
        samples=args.samples,
        output=args.output,
    )
