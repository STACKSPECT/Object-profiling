"""MERGE: retroproyeccion pinhole de profundidad a puntos mundo."""

from __future__ import annotations

import numpy as np

from ..contracts import CameraObservation


def lateral_pitch_m(observation: CameraObservation, mask: np.ndarray) -> float:
    """Tamano lateral de un pixel a la distancia observada.

    Es el suelo con el que puede situarse un borde de silueta, y por tanto un
    limite fisico de la incertidumbre dimensional.
    """

    depths = observation.depth_m[mask]
    valid = depths[np.isfinite(depths) & (depths > 0.0)]
    if valid.size == 0:
        return float("inf")
    return float(np.median(valid) / observation.intrinsics.fx)


def backproject_depth(observation: CameraObservation, mask: np.ndarray) -> np.ndarray:
    """In: RGB-D y mascara. Out: puntos (N, 3) en el marco mundo de la camara."""
    rows, cols = np.nonzero(mask)
    depths = observation.depth_m[rows, cols]
    valid = np.isfinite(depths) & (depths > 0.0)
    rows, cols, depths = rows[valid], cols[valid], depths[valid]
    intrinsics = observation.intrinsics
    x = (cols.astype(np.float64) - intrinsics.cx) * depths / intrinsics.fx
    y = (rows.astype(np.float64) - intrinsics.cy) * depths / intrinsics.fy
    points_camera = np.column_stack([x, y, depths, np.ones_like(depths)])
    return (observation.camera_to_world @ points_camera.T).T[:, :3]
