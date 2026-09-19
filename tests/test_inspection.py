from __future__ import annotations

import numpy as np

from object_profiling.config import AppConfig
from object_profiling.contracts import BoxCondition, Extent3D, RoutingHint, ViewDescriptor
from object_profiling.measure.geometry import CuboidEstimate, FaceCoverage, dimensions_from_extent
from object_profiling.measure.inspection import assess_damage, inspect_cloud
from object_profiling.measure.registration import FusedCloud


def _cloud_from_points(points: np.ndarray) -> FusedCloud:
    return FusedCloud(
        frame_id="ur10e_attachment_site",
        points_m=points,
        view_index=np.zeros(points.shape[0], dtype=np.int32),
        views=(ViewDescriptor("SCAN_YAW_0", 0, 0),),
    )


def _estimate(lower: np.ndarray, upper: np.ndarray) -> CuboidEstimate:
    extent = upper - lower
    coverage = FaceCoverage(0.002, 50, (100, 100, 100), (100, 100, 100))
    return CuboidEstimate(
        dimensions=dimensions_from_extent(extent),
        uncertainty=Extent3D(0.001, 0.001, 0.001),
        confidence=1.0,
        lower_m=lower,
        upper_m=upper,
        points=0,
        residual_p95_m=0.0,
        view_plane_residual_m=0.0,
        view_extent_disagreement_m=np.zeros(3),
        coverage=coverage,
    )


def _face_grid(axis: int, sign: float, lower: np.ndarray, upper: np.ndarray, n: int = 20) -> np.ndarray:
    rng = np.random.default_rng(0)
    points = rng.uniform(lower, upper, size=(n * n, 3))
    coordinate = upper[axis] if sign > 0 else lower[axis]
    points[:, axis] = coordinate
    return points


def test_intact_cloud_is_assessed_intact() -> None:
    lower = np.asarray([-0.15, -0.10, 0.089])
    upper = np.asarray([0.15, 0.10, 0.239])
    faces = [_face_grid(axis, sign, lower, upper, n=40) for axis in range(3) for sign in (-1.0, 1.0)]
    estimate = _estimate(lower, upper)
    metrics = inspect_cloud(_cloud_from_points(np.concatenate(faces)), estimate, AppConfig())
    assessment = assess_damage(metrics, upper - lower, AppConfig())
    assert assessment.condition is BoxCondition.INTACT
    assert assessment.routing is RoutingHint.NORMAL
    assert metrics.max_inward_m < 0.003


def test_missing_corner_is_an_outlier_in_support() -> None:
    lower = np.asarray([-0.15, -0.10, 0.089])
    upper = np.asarray([0.15, 0.10, 0.239])
    faces = [_face_grid(axis, sign, lower, upper, n=40) for axis in range(3) for sign in (-1.0, 1.0)]
    points = np.concatenate(faces)
    missing = np.asarray([upper[0], upper[1], upper[2]])
    points = points[np.linalg.norm(points - missing, axis=1) > 0.05]
    estimate = _estimate(lower, upper)
    metrics = inspect_cloud(_cloud_from_points(points), estimate, AppConfig())
    assert metrics.weakest_corner_support < AppConfig().damage.minimum_corner_support
    assessment = assess_damage(metrics, upper - lower, AppConfig())
    assert assessment.condition is BoxCondition.DAMAGED
    assert assessment.routing is RoutingHint.ERROR_ZONE
    assert assessment.report is not None
    assert "corner:" in assessment.report.location
    assert assessment.report.location.endswith("+z")
