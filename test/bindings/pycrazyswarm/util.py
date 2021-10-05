"""Useful functions for both pycrazyswarm internals and user scripts."""

import numpy as np
import scipy as sp
import scipy.spatial


def check_ellipsoid_collisions(positions, radii):
    """Checks for collisions between a set of ellipsoids at given positions.

    Args:
        positions (array float[n, 3]): The ellipsoid centers.
        radii (array float[3]): The radii of the axis-aligned ellipsoids.

    Returns:
        colliding (array bool[n]): True at index i if the i'th ellipsoid
            intersects any of the other ellipsoids.
    """
    scaled = positions / radii[None, :]
    dists = sp.spatial.distance.pdist(scaled)
    dists = sp.spatial.distance.squareform(dists)
    # Do not consider 0 distance to self as a collision!
    n, _ = positions.shape
    dists[range(n), range(n)] = np.inf
    colliding = np.any(dists < 1.97, axis=1)
    return colliding


def poisson_disk_sample(n, dim, mindist):
    """Generates random points with guaranteed minimum pairwise distance.

    Uses extremely naive and slow "dart throwing" algorithm.
    TODO(jpreiss): find/implement a library with a fast algorithm.

    Args:
        n (int): Number of points.
        dim (int): Dimensionality of points.
        mindist (float): Minimum Euclidean distance between any two points.

    Returns:
        pts (array float[n, dim]): The sampled points.
    """

    # Select hypercube volume such that n points will not pack it too tightly.
    # Note: Will be too sparse for dim >> 3, but reasonable for dim == 2 or 3.
    measure_ratio = 1.25
    std = (measure_ratio * n) ** (1.0 / dim) * mindist
    def sample():
        return std * np.random.uniform(-0.5, 0.5, size=dim)

    # Sample the points using dart-throwing.
    pts = sample()[None,:]
    while len(pts) < n:
        pt = sample()
        dists = np.linalg.norm(pts - pt, axis=1)
        if np.all(dists >= mindist):
            pts = np.concatenate([pts, pt[None,:]], axis=0)
    return pts


def grid_yaml(rows, cols, spacing=0.5):
    """Generate crazyflies.yaml string for a grid in the XZ plane.

    Args:
        rows (int): Number of rows (distinct X-values) in the grid.
        cols (int): Number of columns (distinct Y-values) in the grid.
        spacing (float): Grid spacing for both axes.

    Returns:
        yaml (str): String containing crazyflies.yaml. Will contain
            (rows * cols) crazyflies, all assigned on radio channel 100.
            Positions will be centered about the origin. No particular
            id-position mapping should be assumed.
    """
    x, y = spacing * np.mgrid[:rows, :cols]
    x -= np.mean(x)
    y -= np.mean(y)
    return "crazyflies:\n" + "\n".join([
        """- channel: 100
  id: {}
  initialPosition: [{}, {}, 0.0]""".format(i, x, y)
        for i, (x, y) in enumerate(zip(x.flat, y.flat))
    ])


