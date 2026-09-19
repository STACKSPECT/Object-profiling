from __future__ import annotations

import numpy as np
import pytest

from object_profiling.contracts import ScanView
from object_profiling.registration import (
    TOOL_FRAME_ID,
    axis_aligned_extremes_m,
    distance_to_box_surface_m,
    fuse_scan_views,
    view_extent_disagreement_m,
    view_plane_residuals_m,
)
from object_profiling.registration_audit import audit_registration_suite
from object_profiling.sensors import backproject_depth
from object_profiling.contracts import CameraIntrinsics, CameraObservation


def _view(pose_name: str, yaw: int, tilt: int, points: np.ndarray) -> ScanView:
    empty = np.zeros((2, 2), dtype=np.uint8)
    return ScanView(pose_name, yaw, tilt, empty, empty.astype(np.float64), empty.astype(bool), points, False)


def _synthetic_observation(camera_to_world: np.ndarray, depth: np.ndarray) -> CameraObservation:
    height, width = depth.shape
    intrinsics = CameraIntrinsics(width, height, 100.0, 100.0, (width - 1) / 2.0, (height - 1) / 2.0)
    return CameraObservation(
        timestamp_s=0.0,
        pose_name="SYNTHETIC",
        target_yaw_deg=0,
        target_tilt_deg=0,
        rgb=np.zeros((height, width, 3), dtype=np.uint8),
        depth_m=depth,
        intrinsics=intrinsics,
        camera_to_world=camera_to_world,
        tool_to_world=np.eye(4),
    )


def test_backprojection_places_the_principal_point_on_the_optical_axis() -> None:
    depth = np.full((5, 5), 2.0)
    observation = _synthetic_observation(np.eye(4), depth)
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True

    points = backproject_depth(observation, mask)

    assert points.shape == (1, 3)
    assert points[0] == pytest.approx([0.0, 0.0, 2.0])


def test_backprojection_scales_offsets_with_depth_and_focal_length() -> None:
    depth = np.full((5, 5), 3.0)
    observation = _synthetic_observation(np.eye(4), depth)
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 4] = True

    points = backproject_depth(observation, mask)

    # Dos pixeles a la derecha del centro, con fx = 100 y profundidad 3 m.
    assert points[0] == pytest.approx([2.0 * 3.0 / 100.0, 0.0, 3.0])


def test_backprojection_applies_the_camera_to_world_transform() -> None:
    camera_to_world = np.eye(4)
    camera_to_world[:3, 3] = [1.0, -2.0, 0.5]
    depth = np.full((3, 3), 1.0)
    observation = _synthetic_observation(camera_to_world, depth)
    mask = np.zeros((3, 3), dtype=bool)
    mask[1, 1] = True

    points = backproject_depth(observation, mask)

    assert points[0] == pytest.approx([1.0, -2.0, 1.5])


def test_backprojection_drops_invalid_depth() -> None:
    depth = np.asarray([[1.0, 0.0], [np.nan, 2.0]])
    observation = _synthetic_observation(np.eye(4), depth)

    points = backproject_depth(observation, np.ones((2, 2), dtype=bool))

    assert points.shape[0] == 2


def test_fusion_keeps_track_of_which_view_each_point_came_from() -> None:
    first = _view("SCAN_YAW_0", 0, 0, np.zeros((4, 3)))
    second = _view("SCAN_TILT_35", 0, 35, np.ones((7, 3)))

    cloud = fuse_scan_views([first, second])

    assert cloud.frame_id == TOOL_FRAME_ID
    assert cloud.points_m.shape == (11, 3)
    assert cloud.view_count == 2
    assert [view.pose_name for view in cloud.views] == ["SCAN_YAW_0", "SCAN_TILT_35"]
    assert cloud.points_for_view(0).shape == (4, 3)
    assert cloud.points_for_view(1).shape == (7, 3)
    assert [view.tilt_deg for view in cloud.views] == [0, 35]


def test_fusion_rejects_an_empty_view_list() -> None:
    with pytest.raises(ValueError):
        fuse_scan_views([])


def test_distance_to_box_surface_is_zero_on_the_face() -> None:
    lower, upper = np.asarray([-1.0, -1.0, -1.0]), np.asarray([1.0, 1.0, 1.0])
    points = np.asarray([[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 0.0, 0.5]])

    distance = distance_to_box_surface_m(points, lower, upper)

    assert distance[0] == pytest.approx(0.0)
    assert distance[1] == pytest.approx(1.0)
    assert distance[2] == pytest.approx(1.0)
    assert distance[3] == pytest.approx(0.5)


def test_axis_aligned_extremes_use_robust_percentiles() -> None:
    points = np.zeros((1000, 3))
    points[:, 0] = np.linspace(-0.1, 0.1, 1000)
    points[0, 0] = -5.0

    lower, upper = axis_aligned_extremes_m(points, 0.5, 99.5)

    assert lower[0] > -0.2
    assert upper[0] == pytest.approx(0.1, abs=0.002)


def test_a_misregistered_view_raises_its_plane_residual() -> None:
    rng = np.random.default_rng(7)
    face = np.column_stack(
        [rng.uniform(-0.1, 0.1, 400), rng.uniform(-0.05, 0.05, 400), np.full(400, 0.075)]
    )
    aligned = _view("SCAN_YAW_0", 0, 0, face)
    shifted = _view("SCAN_YAW_90", 90, 0, face + np.asarray([0.0, 0.0, 0.02]))

    good = view_plane_residuals_m(fuse_scan_views([aligned, aligned]), 0.5, 99.5)
    bad = view_plane_residuals_m(fuse_scan_views([aligned, shifted]), 0.5, 99.5)

    assert float(np.max(good)) < 1e-6
    assert float(np.max(bad)) > float(np.max(good))


def test_extent_disagreement_is_zero_for_identical_views() -> None:
    rng = np.random.default_rng(11)
    points = rng.uniform(-0.1, 0.1, (300, 3))
    cloud = fuse_scan_views([_view("A", 0, 0, points), _view("B", 90, 0, points)])

    assert view_extent_disagreement_m(cloud) == pytest.approx(np.zeros(3))


def test_three_views_register_into_a_single_rigid_cloud() -> None:
    """La caja esta soldada al terminal, asi que las tres nubes deben coincidir."""

    report = audit_registration_suite()

    assert report["valid"] is True
    assert report["frame_id"] == TOOL_FRAME_ID
    assert len(report["records"]) == 3
    for record in report["records"]:
        assert record["views_used"] == ("SCAN_YAW_0", "SCAN_YAW_90", "SCAN_TILT_35")
        assert record["fused_points"] > 10_000
        # Cada vista por separado se retroproyecta con error de micras.
        for per_view in record["per_view_registration"]:
            assert per_view["p95_surface_distance_m"] < 1e-4
        # La fusion anade el desplazamiento residual de la caja entre poses.
        assert record["fused_registration"]["p95_surface_distance_m"] < 1e-3
