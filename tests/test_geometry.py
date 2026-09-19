from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from object_profiling.config import AppConfig, EstimatorConfig
from object_profiling.contracts import Dimensions3D, RejectionReason
from object_profiling.measure.perception import ScanView
from object_profiling.measure.geometry import (
    FaceCoverage,
    combine_uncertainty_m,
    estimate_cuboid,
    face_coverage,
)
from object_profiling.evaluation.audits.geometry import METHODS, audit_geometry_suite
from object_profiling.measure.registration import fuse_scan_views

CONFIG = AppConfig()
TOP_Z_M = CONFIG.sensor.tool_to_box_offset_m
NOMINAL_PITCH_M = 0.0016


def _face_points(dimensions: np.ndarray, per_face: int, seed: int) -> np.ndarray:
    """Muestrea las seis caras de un cuboide colgado del marco del terminal."""

    rng = np.random.default_rng(seed)
    length, width, height = dimensions
    half = np.asarray([length / 2.0, width / 2.0])
    clouds = []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            point = np.empty((per_face, 3))
            point[:, 0] = rng.uniform(-half[0], half[0], per_face)
            point[:, 1] = rng.uniform(-half[1], half[1], per_face)
            point[:, 2] = rng.uniform(TOP_Z_M, TOP_Z_M + height, per_face)
            if axis == 0:
                point[:, 0] = sign * half[0]
            elif axis == 1:
                point[:, 1] = sign * half[1]
            else:
                point[:, 2] = TOP_Z_M if sign < 0 else TOP_Z_M + height
            clouds.append(point)
    return np.concatenate(clouds, axis=0)


def _view(pose_name: str, yaw: int, tilt: int, points: np.ndarray) -> ScanView:
    empty = np.zeros((2, 2))
    return ScanView(pose_name, yaw, tilt, empty.astype(np.uint8), empty, empty.astype(bool), points, False)


POSE_NAMES = ("SCAN_YAW_0", "SCAN_YAW_90", "SCAN_TILT_35")


def _cloud(points: np.ndarray, views: int = 3):
    # Se reparte por saltos para que cada vista toque todas las caras, como
    # ocurre en la escena real.
    return fuse_scan_views(
        [_view(POSE_NAMES[index], 0, 0, points[index::views]) for index in range(views)]
    )


def _estimate(points: np.ndarray, *, views: int = 3, pitch: float = NOMINAL_PITCH_M, config=CONFIG):
    return estimate_cuboid(_cloud(points, views), config, seed=42, lateral_pitch_m=pitch)


def test_length_is_the_larger_horizontal_extent() -> None:
    # Caja mas ancha que larga en el marco del terminal: la convencion reordena.
    points = _face_points(np.asarray([0.18, 0.32, 0.12]), 900, seed=1)

    estimate, reason = _estimate(points)

    assert reason is None
    assert estimate.dimensions.length == pytest.approx(0.32, abs=0.002)
    assert estimate.dimensions.width == pytest.approx(0.18, abs=0.002)
    assert estimate.dimensions.height == pytest.approx(0.12, abs=0.002)


def test_a_clean_cuboid_is_recovered_within_a_millimetre() -> None:
    truth = np.asarray([0.31, 0.22, 0.17])
    points = _face_points(truth, 1200, seed=2)

    estimate, reason = _estimate(points)

    assert reason is None
    assert estimate.dimensions.as_array() == pytest.approx(truth, abs=0.001)
    assert estimate.points == points.shape[0]
    assert 0.0 <= estimate.confidence <= 1.0


def test_face_coverage_counts_support_on_both_ends_of_each_axis() -> None:
    points = _face_points(np.asarray([0.30, 0.20, 0.15]), 1000, seed=3)
    lower, upper = points.min(axis=0), points.max(axis=0)

    coverage = face_coverage(points, lower, upper, CONFIG)

    assert coverage.slab_m == CONFIG.estimator.coverage_slab_m
    assert coverage.satisfied
    assert coverage.weakest_support >= CONFIG.estimator.minimum_face_support_points


def test_a_sparsely_supported_extreme_is_rejected() -> None:
    """Un extremo sostenido por unos pocos puntos no puede pasar por medida."""

    points = _face_points(np.asarray([0.30, 0.20, 0.15]), 1200, seed=4)
    # Se retira la cara +y y se deja un punado de puntos 6 mm mas alla, de modo
    # que el extremo robusto caiga sobre ellos sin apoyo real.
    body = points[points[:, 1] < 0.20 / 2.0 - 0.001]
    sparse = np.column_stack(
        [
            np.linspace(-0.1, 0.1, 8),
            np.full(8, 0.20 / 2.0 + 0.006),
            np.linspace(TOP_Z_M, TOP_Z_M + 0.15, 8),
        ]
    )

    estimate, reason = _estimate(np.concatenate([body, sparse]))

    assert estimate is None
    assert reason is RejectionReason.INSUFFICIENT_FACE_COVERAGE


def test_dimensions_outside_the_declared_range_are_rejected() -> None:
    points = _face_points(np.asarray([0.60, 0.50, 0.40]), 900, seed=5)

    estimate, reason = _estimate(points)

    assert estimate is None
    assert reason is RejectionReason.OUT_OF_RANGE


def test_fewer_views_than_the_fixed_sequence_are_rejected() -> None:
    points = _face_points(np.asarray([0.30, 0.20, 0.15]), 900, seed=6)

    estimate, reason = _estimate(points, views=2)

    assert estimate is None
    assert reason is RejectionReason.INSUFFICIENT_VIEWS


def test_too_few_points_are_rejected() -> None:
    points = _face_points(np.asarray([0.30, 0.20, 0.15]), 20, seed=7)

    estimate, reason = _estimate(points)

    assert estimate is None
    assert reason is RejectionReason.INSUFFICIENT_FOREGROUND


def test_a_misregistered_view_is_rejected() -> None:
    points = _face_points(np.asarray([0.30, 0.20, 0.15]), 1200, seed=8)
    chunks = [points[index::3] for index in range(3)]
    chunks[2] = chunks[2] + np.asarray([0.0, 0.0, 0.01])
    cloud = fuse_scan_views(
        [_view(POSE_NAMES[index], 0, 0, chunk) for index, chunk in enumerate(chunks)]
    )

    estimate, reason = estimate_cuboid(cloud, CONFIG, seed=42, lateral_pitch_m=NOMINAL_PITCH_M)

    assert estimate is None
    assert reason is RejectionReason.REGISTRATION_INCONSISTENT


def test_a_coarse_camera_is_rejected_for_high_uncertainty() -> None:
    points = _face_points(np.asarray([0.30, 0.20, 0.15]), 1200, seed=9)

    estimate, reason = _estimate(points, pitch=0.05)

    assert estimate is None
    assert reason is RejectionReason.HIGH_UNCERTAINTY


def test_uncertainty_grows_with_lateral_resolution_and_shrinks_with_support() -> None:
    bootstrap = np.full(3, 0.0002)
    strong = FaceCoverage(0.002, 50, (500, 500, 500), (500, 500, 500))
    weak = FaceCoverage(0.002, 50, (500, 5, 500), (500, 500, 500))

    fine = combine_uncertainty_m(bootstrap, 0.001, 0.0002, 0.00005, strong)
    coarse = combine_uncertainty_m(bootstrap, 0.004, 0.0002, 0.00005, strong)
    starved = combine_uncertainty_m(bootstrap, 0.001, 0.0002, 0.00005, weak)

    assert np.all(coarse > fine)
    assert starved[1] > fine[1]
    assert starved[0] == pytest.approx(fine[0])


def test_estimation_is_deterministic_for_a_given_seed() -> None:
    points = _face_points(np.asarray([0.30, 0.20, 0.15]), 1200, seed=10)

    first, _ = _estimate(points)
    second, _ = _estimate(points)

    assert first.dimensions == second.dimensions
    assert first.uncertainty == second.uncertainty
    assert first.confidence == second.confidence


def test_a_larger_bootstrap_budget_does_not_change_the_dimensions() -> None:
    points = _face_points(np.asarray([0.30, 0.20, 0.15]), 1200, seed=11)
    wider = AppConfig(estimator=dataclasses.replace(EstimatorConfig(), bootstrap_samples=48))

    baseline, _ = _estimate(points)
    extended, _ = _estimate(points, config=wider)

    assert baseline.dimensions == extended.dimensions


def test_the_convention_rejects_a_width_larger_than_the_length() -> None:
    with pytest.raises(ValueError):
        Dimensions3D(0.10, 0.20, 0.15)


def test_rendered_boxes_are_measured_below_a_millimetre_and_uncertainty_covers_it() -> None:
    report = audit_geometry_suite(seed=42)

    assert report["valid"] is True
    assert report["worst_absolute_error_mm"] < 1.0
    assert report["uncertainty_covers_error"] is True
    for record in report["records"]:
        assert record["rejection_reason"] is None
        assert record["weakest_face_support"] >= CONFIG.estimator.minimum_face_support_points


def test_the_axis_aligned_baseline_beats_the_alternatives() -> None:
    """La comparacion sostiene la eleccion del baseline con numeros."""

    report = audit_geometry_suite(seed=42)
    worst = report["worst_error_by_method_mm"]

    assert set(worst) == set(METHODS)
    baseline = worst["robust_extents"]
    assert baseline <= worst["raw_extents"]
    assert baseline <= worst["trimmed_extents_0p5"]
    assert baseline <= worst["plane_refined"]
    # Las componentes principales no recuperan los ejes de la caja: la nube esta
    # muy sesgada hacia las caras que la camara ve mejor.
    assert worst["principal_axes"] > 10.0
