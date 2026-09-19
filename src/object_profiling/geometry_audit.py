from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

from .checkpoint import MAXIMUM_BOX, MINIMUM_BOX, NOMINAL_BOX
from .config import AppConfig
from .contracts import BoxSpec, Dimensions3D
from .environment import ProfilingEnvironment
from .geometry import estimate_cuboid
from .perception import observation_to_scan_view
from .registration import FusedCloud, axis_aligned_extremes_m, fuse_scan_views
from .scanning import run_fixed_scan
from .sensors import RGBDSensor, lateral_pitch_m


def _dimensions(extent: np.ndarray) -> np.ndarray:
    horizontal = sorted((float(extent[0]), float(extent[1])), reverse=True)
    return np.asarray([horizontal[0], horizontal[1], float(extent[2])])


def method_robust_extents(cloud: FusedCloud, config: AppConfig) -> np.ndarray:
    lower, upper = axis_aligned_extremes_m(
        cloud.points_m, config.estimator.percentile_low, config.estimator.percentile_high
    )
    return _dimensions(upper - lower)


def method_raw_extents(cloud: FusedCloud, _config: AppConfig) -> np.ndarray:
    return _dimensions(np.ptp(cloud.points_m, axis=0))


def method_trimmed_extents(cloud: FusedCloud, _config: AppConfig) -> np.ndarray:
    """El recorte del prototipo heredado, medio punto porcentual por extremo."""

    lower, upper = axis_aligned_extremes_m(cloud.points_m, 0.5, 99.5)
    return _dimensions(upper - lower)


def method_min_area_rect(cloud: FusedCloud, config: AppConfig) -> np.ndarray:
    """Rectangulo minimo sobre el convex hull en XY, como el prototipo."""

    points = cloud.points_m
    hull = cv2.convexHull(points[:, :2].astype(np.float32))
    (_center, (width, height), _angle) = cv2.minAreaRect(hull)
    low, high = config.estimator.percentile_low, config.estimator.percentile_high
    vertical = float(np.percentile(points[:, 2], high) - np.percentile(points[:, 2], low))
    return _dimensions(np.asarray([width, height, vertical]))


def method_principal_axes(cloud: FusedCloud, config: AppConfig) -> np.ndarray:
    """OBB por componentes principales en el plano horizontal."""

    points = cloud.points_m
    horizontal = points[:, :2] - points[:, :2].mean(axis=0)
    _values, vectors = np.linalg.eigh(np.cov(horizontal, rowvar=False))
    projected = horizontal @ vectors
    low, high = config.estimator.percentile_low, config.estimator.percentile_high
    extent = np.percentile(projected, high, axis=0) - np.percentile(projected, low, axis=0)
    vertical = float(np.percentile(points[:, 2], high) - np.percentile(points[:, 2], low))
    return _dimensions(np.asarray([extent[0], extent[1], vertical]))


def method_plane_refined(cloud: FusedCloud, config: AppConfig) -> np.ndarray:
    """Refina cada cara con la mediana de sus puntos de soporte.

    Evita que un extremo quede fijado por el punto mas alejado: usa el plano que
    mejor describe la loncha de puntos que lo sostiene.
    """

    points = cloud.points_m
    lower, upper = axis_aligned_extremes_m(
        cloud.points_m, config.estimator.percentile_low, config.estimator.percentile_high
    )
    slab = config.estimator.coverage_slab_m
    refined_lower = np.empty(3)
    refined_upper = np.empty(3)
    for axis in range(3):
        column = points[:, axis]
        near_lower = column[column <= lower[axis] + slab]
        near_upper = column[column >= upper[axis] - slab]
        refined_lower[axis] = np.median(near_lower) if near_lower.size else lower[axis]
        refined_upper[axis] = np.median(near_upper) if near_upper.size else upper[axis]
    return _dimensions(refined_upper - refined_lower)


METHODS = {
    "robust_extents": method_robust_extents,
    "raw_extents": method_raw_extents,
    "trimmed_extents_0p5": method_trimmed_extents,
    "min_area_rect_hull": method_min_area_rect,
    "principal_axes": method_principal_axes,
    "plane_refined": method_plane_refined,
}


@dataclass(frozen=True)
class MethodError:
    method: str
    length_error_mm: float
    width_error_mm: float
    height_error_mm: float
    maximum_absolute_error_mm: float


@dataclass(frozen=True)
class GeometryRecord:
    object_id: str
    ground_truth_m: tuple[float, float, float]
    estimated_m: tuple[float, float, float]
    uncertainty_m: tuple[float, float, float]
    absolute_error_mm: tuple[float, float, float]
    confidence: float
    points: int
    residual_p95_mm: float
    view_plane_residual_mm: float
    weakest_face_support: int
    lateral_pitch_mm: float
    rejection_reason: str | None
    uncertainty_covers_error: bool
    method_errors: tuple[MethodError, ...]


def audit_box_geometry(
    box_spec: BoxSpec,
    *,
    seed: int = 42,
    config: AppConfig | None = None,
) -> GeometryRecord:
    config = config or AppConfig()
    environment = ProfilingEnvironment.create(box_spec, config, attach_box=False)
    sensor = RGBDSensor(environment)
    views = []
    pitches: list[float] = []

    def on_capture(pose, observation, backgrounds) -> None:
        view, _reason = observation_to_scan_view(observation, backgrounds.depth_for(pose.name), config)
        if view is None:
            return
        views.append(view)
        pitches.append(lateral_pitch_m(observation, view.mask))

    try:
        run_fixed_scan(environment, sensor, on_capture=on_capture)
    finally:
        sensor.close()

    cloud = fuse_scan_views(views)
    pitch = float(np.median(pitches))
    estimate, reason = estimate_cuboid(cloud, config, seed=seed, lateral_pitch_m=pitch)
    truth = box_spec.dimensions_m.as_array()

    method_errors = tuple(
        _method_error(name, method(cloud, config), truth) for name, method in METHODS.items()
    )
    if estimate is None:
        return GeometryRecord(
            object_id=box_spec.object_id,
            ground_truth_m=tuple(float(value) for value in truth),
            estimated_m=(0.0, 0.0, 0.0),
            uncertainty_m=(0.0, 0.0, 0.0),
            absolute_error_mm=(0.0, 0.0, 0.0),
            confidence=0.0,
            points=int(cloud.points_m.shape[0]),
            residual_p95_mm=0.0,
            view_plane_residual_mm=0.0,
            weakest_face_support=0,
            lateral_pitch_mm=pitch * 1000.0,
            rejection_reason=reason.value if reason else None,
            uncertainty_covers_error=False,
            method_errors=method_errors,
        )

    estimated = estimate.dimensions.as_array()
    uncertainty = estimate.uncertainty.as_array()
    error = np.abs(estimated - truth)
    return GeometryRecord(
        object_id=box_spec.object_id,
        ground_truth_m=tuple(float(value) for value in truth),
        estimated_m=tuple(float(value) for value in estimated),
        uncertainty_m=tuple(float(value) for value in uncertainty),
        absolute_error_mm=tuple(float(value) * 1000.0 for value in error),
        confidence=estimate.confidence,
        points=estimate.points,
        residual_p95_mm=estimate.residual_p95_m * 1000.0,
        view_plane_residual_mm=estimate.view_plane_residual_m * 1000.0,
        weakest_face_support=estimate.coverage.weakest_support,
        lateral_pitch_mm=pitch * 1000.0,
        rejection_reason=None,
        uncertainty_covers_error=bool(np.all(error <= uncertainty)),
        method_errors=method_errors,
    )


def _method_error(name: str, estimated: np.ndarray, truth: np.ndarray) -> MethodError:
    error = (estimated - truth) * 1000.0
    return MethodError(
        method=name,
        length_error_mm=float(error[0]),
        width_error_mm=float(error[1]),
        height_error_mm=float(error[2]),
        maximum_absolute_error_mm=float(np.max(np.abs(error))),
    )


def audit_geometry_suite(*, seed: int = 42) -> dict:
    records = [
        audit_box_geometry(box_spec, seed=seed)
        for box_spec in (MINIMUM_BOX, NOMINAL_BOX, MAXIMUM_BOX)
    ]
    by_method: dict[str, float] = {}
    for record in records:
        for method_error in record.method_errors:
            by_method[method_error.method] = max(
                by_method.get(method_error.method, 0.0), method_error.maximum_absolute_error_mm
            )
    return {
        "schema_version": 1,
        "checkpoint": "cuboid_estimation",
        "valid": all(record.rejection_reason is None for record in records),
        "worst_absolute_error_mm": max(max(record.absolute_error_mm) for record in records),
        "uncertainty_covers_error": all(record.uncertainty_covers_error for record in records),
        "worst_error_by_method_mm": by_method,
        "records": [asdict(record) for record in records],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compara el estimador de cuboide contra metodos alternativos."
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", type=Path, help="Ruta opcional para el informe JSON.")
    args = parser.parse_args()

    report = audit_geometry_suite(seed=args.seed)
    payload = json.dumps(report, indent=2, sort_keys=True)
    print(payload)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
