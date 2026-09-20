from __future__ import annotations

import numpy as np
import pytest

from object_profiling.config import AppConfig
from object_profiling.contracts import RejectionReason
from object_profiling.measure.perception import (
    segment_foreground,
    tool_volume_bounds_m,
)

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
    assert lower[2] == pytest.approx(offset - CONFIG.sensor.cup_plane_margin_m)
    assert upper[2] == pytest.approx(offset + CONFIG.box_range.height_m[1] + margin)


def test_the_crop_excludes_the_suction_cups() -> None:
    """Las copas ocupan z entre 0,065 y 0,089 en el marco del terminal."""

    lower, _upper = tool_volume_bounds_m(CONFIG)

    assert lower[2] > 0.080
