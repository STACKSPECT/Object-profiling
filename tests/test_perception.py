from __future__ import annotations

import numpy as np
import pytest

from object_profiling.checkpoint import NOMINAL_BOX
from object_profiling.config import AppConfig
from object_profiling.contracts import RejectionReason
from object_profiling.environment import ProfilingEnvironment
from object_profiling.evaluation import GroundTruthRenderer, evaluate_segmentation
from object_profiling.perception import (
    observation_to_scan_view,
    points_in_tool_frame,
    segment_foreground,
    tool_volume_bounds_m,
)
from object_profiling.scanning import run_fixed_scan
from object_profiling.segmentation_audit import audit_segmentation_suite
from object_profiling.sensors import RGBDSensor

CONFIG = AppConfig()
FAR_BACKGROUND_M = 2.0


def _background() -> np.ndarray:
    return np.full((CONFIG.sensor.height, CONFIG.sensor.width), FAR_BACKGROUND_M)


def _depth_with_patch(top: int, left: int, height: int, width: int, depth_m: float = 1.0) -> np.ndarray:
    depth = _background()
    depth[top : top + height, left : left + width] = depth_m
    return depth


def test_empty_foreground_is_rejected_explicitly() -> None:
    result = segment_foreground(_background(), _background(), CONFIG.sensor)

    assert result.reason is RejectionReason.INSUFFICIENT_FOREGROUND
    assert not result.mask.any()
    assert not result.interior_mask.any()


def test_depth_difference_below_the_margin_is_not_foreground() -> None:
    barely_closer = FAR_BACKGROUND_M - CONFIG.sensor.foreground_margin_m / 2.0
    depth = _depth_with_patch(200, 200, 80, 80, depth_m=barely_closer)

    result = segment_foreground(depth, _background(), CONFIG.sensor)

    assert result.reason is RejectionReason.INSUFFICIENT_FOREGROUND


def test_component_smaller_than_the_threshold_is_rejected() -> None:
    depth = _depth_with_patch(200, 200, 30, 30)

    result = segment_foreground(depth, _background(), CONFIG.sensor)

    assert result.reason is RejectionReason.INSUFFICIENT_FOREGROUND
    assert int(np.count_nonzero(result.mask)) < CONFIG.sensor.min_component_pixels


def test_component_touching_the_frame_border_is_rejected() -> None:
    depth = _depth_with_patch(0, 200, 120, 120)

    result = segment_foreground(depth, _background(), CONFIG.sensor)

    assert result.reason is RejectionReason.FRAME_BORDER_CONTACT
    assert result.touches_border


def test_largest_component_wins_and_zero_erosion_keeps_the_full_mask() -> None:
    depth = _depth_with_patch(150, 150, 100, 100)
    depth[400:460, 500:560] = 1.2

    result = segment_foreground(depth, _background(), CONFIG.sensor)

    assert result.reason is None
    assert int(np.count_nonzero(result.mask)) == 100 * 100
    interior = int(np.count_nonzero(result.interior_mask))
    expected = (100 - 2 * CONFIG.sensor.mask_erosion_px) ** 2
    assert interior == expected
    assert CONFIG.sensor.mask_erosion_px == 0
    assert np.array_equal(result.interior_mask, result.mask)


def test_tool_volume_is_derived_from_the_declared_box_range() -> None:
    lower, upper = tool_volume_bounds_m(CONFIG)
    margin = CONFIG.sensor.tool_volume_margin_m
    offset = CONFIG.sensor.tool_to_box_offset_m

    horizontal = max(CONFIG.box_range.length_m[1], CONFIG.box_range.width_m[1]) / 2.0 + margin
    assert lower[:2] == pytest.approx([-horizontal, -horizontal])
    assert upper[:2] == pytest.approx([horizontal, horizontal])
    # El limite superior en z es el plano de contacto de las copas, con holgura
    # minima: un margen generoso metia las propias copas en la nube.
    assert lower[2] == pytest.approx(offset - CONFIG.sensor.cup_plane_margin_m)
    assert upper[2] == pytest.approx(offset + CONFIG.box_range.height_m[1] + margin)


def test_the_crop_excludes_the_suction_cups() -> None:
    """Las copas ocupan z entre 0,065 y 0,089 en el marco del terminal."""

    lower, _upper = tool_volume_bounds_m(CONFIG)

    assert lower[2] > 0.080


@pytest.fixture(scope="module")
def nominal_cycle():
    environment = ProfilingEnvironment.create(NOMINAL_BOX, CONFIG, attach_box=False)
    sensor = RGBDSensor(environment)
    truth_renderer = GroundTruthRenderer(environment)
    truths: dict[str, object] = {}

    def on_capture(pose, _observation, _backgrounds) -> None:
        truths[pose.name] = truth_renderer.masks()

    try:
        cycle = run_fixed_scan(environment, sensor, on_capture=on_capture)
        yield cycle, truths
    finally:
        sensor.close()
        truth_renderer.close()


def test_observable_points_land_inside_the_tool_volume(nominal_cycle) -> None:
    cycle, _truths = nominal_cycle
    lower, upper = tool_volume_bounds_m(CONFIG)

    for observation in cycle.observations:
        background = cycle.backgrounds.depth_for(observation.pose_name)
        segmentation = segment_foreground(observation.depth_m, background, CONFIG.sensor)
        points = points_in_tool_frame(observation, segmentation.interior_mask, CONFIG)

        assert points.shape[0] > 1_000
        assert np.all(points >= lower)
        assert np.all(points <= upper)


def test_scan_views_are_produced_for_every_pose(nominal_cycle) -> None:
    cycle, _truths = nominal_cycle

    for observation in cycle.observations:
        background = cycle.backgrounds.depth_for(observation.pose_name)
        view, reason = observation_to_scan_view(observation, background, CONFIG)

        assert reason is None
        assert view is not None
        assert view.pose_name == observation.pose_name
        assert not view.touches_border
        assert view.points_tool_m.shape[1] == 3


def test_background_from_another_pose_degrades_segmentation(nominal_cycle) -> None:
    """Un fondo equivocado no puede pasar desapercibido."""

    cycle, truths = nominal_cycle
    tilt = next(o for o in cycle.observations if o.pose_name == "SCAN_TILT_35")

    correct = segment_foreground(
        tilt.depth_m, cycle.backgrounds.depth_for("SCAN_TILT_35"), CONFIG.sensor
    )
    swapped = segment_foreground(
        tilt.depth_m, cycle.backgrounds.depth_for("SCAN_YAW_0"), CONFIG.sensor
    )
    truth = truths["SCAN_TILT_35"]

    correct_iou = evaluate_segmentation(correct.mask, truth).intersection_over_union
    swapped_iou = evaluate_segmentation(swapped.mask, truth).intersection_over_union

    assert correct_iou > 0.99
    assert swapped_iou < correct_iou - 0.05


def test_segmentation_is_deterministic(nominal_cycle) -> None:
    cycle, _truths = nominal_cycle
    observation = cycle.observations[0]
    background = cycle.backgrounds.depth_for(observation.pose_name)

    first = segment_foreground(observation.depth_m, background, CONFIG.sensor)
    second = segment_foreground(observation.depth_m, background, CONFIG.sensor)

    np.testing.assert_array_equal(first.mask, second.mask)
    np.testing.assert_array_equal(first.interior_mask, second.interior_mask)


def test_observable_segmentation_matches_ground_truth_across_range_and_poses() -> None:
    """Nueve combinaciones de tamano y pose, sin usar el ID de box_geom."""

    report = audit_segmentation_suite()

    assert report["valid"] is True
    assert len(report["records"]) == 9
    assert report["worst_intersection_over_union"] > 0.99
    assert report["worst_precision"] > 0.99
    assert report["worst_recall"] > 0.99
    # Los unicos falsos positivos son el anillo de silueta de las copas.
    assert all(record["metrics"]["false_other_pixels"] == 0 for record in report["records"])
