"""SolidWorks transform conventions used by the URDF converter.

Component matrices in swTransform are stored for row vectors (p' = p @ M):
the translation is the last row, and the upper 3x3 is the transpose of the
column-vector rotation. Using that block without transposing rotates every
nested part the wrong way. A part whose geometry is far from its origin
then lands metres away from the arm.
"""

from __future__ import annotations

import numpy as np

MM_TO_M = 0.001


def sw16_to_se3(values: list[float]) -> np.ndarray:
    """16 SolidWorks floats to a column-vector SE(3) matrix."""
    raw = np.asarray(values, dtype=float).reshape(4, 4)
    transform = np.eye(4)
    transform[:3, :3] = raw[:3, :3].T
    transform[:3, 3] = raw[3, :3]
    return transform


def cs13_to_se3(values: list[float]) -> np.ndarray:
    """Coordinate system: row-major 3x3, then translation, then a trailing 1."""
    raw = np.asarray(values, dtype=float)
    transform = np.eye(4)
    transform[:3, :3] = raw[:9].reshape(3, 3)
    transform[:3, 3] = raw[9:12]
    return orthonormalize(transform)


def orthonormalize(transform: np.ndarray) -> np.ndarray:
    """Snap a nearly-orthogonal 3x3 to a proper rotation."""
    rotation = transform[:3, :3]
    u, _, vt = np.linalg.svd(rotation)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        u = u.copy()
        u[:, -1] *= -1
        rotation = u @ vt
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = transform[:3, 3]
    return out


def part_axis(world: np.ndarray, column: int) -> np.ndarray:
    """Unit column of a part's world rotation. Shafts use column 1, racks column 0."""
    axis = np.asarray(world[:3, column], dtype=float)
    return axis / np.linalg.norm(axis)
