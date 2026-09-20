from __future__ import annotations

import numpy as np
import pytest

from object_profiling.config import AppConfig
from object_profiling.contracts import (
    OBJECT_DIMENSIONS_SCHEMA_VERSION,
    BoxCondition,
    Dimensions3D,
    Extent3D,
    HeldBoxHandoff,
    ObjectDimensions,
    RejectionReason,
    RoutingHint,
    ViewDescriptor,
    snap_to_catalogue,
)


def test_config_has_no_robot_or_scene_fields() -> None:
    config = AppConfig()

    assert not hasattr(config, "motion")
    assert not hasattr(config, "required_views_deg")
    assert not hasattr(config, "fallback_view_deg")


def test_dimensions_enforce_length_width_convention() -> None:
    dimensions = Dimensions3D(length=0.30, width=0.20, height=0.15)

    assert dimensions.as_array() == pytest.approx(np.asarray([0.30, 0.20, 0.15]))
    with pytest.raises(ValueError):
        Dimensions3D(length=0.20, width=0.30, height=0.15)


def test_snap_to_catalogue_recovers_the_5mm_grid() -> None:
    snapped = snap_to_catalogue(Dimensions3D(0.2474, 0.2431, 0.2466), 0.005)

    assert snapped.as_array() == pytest.approx(np.asarray([0.245, 0.245, 0.245]))
    assert snap_to_catalogue(Dimensions3D(0.30, 0.20, 0.15), 0.0) is None


def test_extent_allows_uncertainty_smaller_on_length_than_width() -> None:
    uncertainty = Extent3D(length=0.001, width=0.004, height=0.002)

    assert uncertainty.as_array() == pytest.approx(np.asarray([0.001, 0.004, 0.002]))


def test_object_dimensions_serializes_structured_views() -> None:
    result = ObjectDimensions(
        object_id="box-0042",
        timestamp_s=1.25,
        frame_id="ur10e_attachment_site",
        dimensions_m=Dimensions3D(0.30, 0.20, 0.15),
        dimensions_snapped_m=Dimensions3D(0.30, 0.20, 0.15),
        uncertainty_m=Extent3D(0.001, 0.002, 0.001),
        pose=None,
        views_used=(
            ViewDescriptor("SCAN_YAW_0", 0, 0),
            ViewDescriptor("SCAN_YAW_90", 90, 0),
        ),
        confidence=0.9,
        valid=True,
        rejection_reason=None,
    )
    payload = result.to_dict()

    assert payload["schema_version"] == OBJECT_DIMENSIONS_SCHEMA_VERSION
    assert "views_used_deg" not in payload
    assert payload["views_used"] == [
        {"pose_name": "SCAN_YAW_0", "yaw_deg": 0, "tilt_deg": 0},
        {"pose_name": "SCAN_YAW_90", "yaw_deg": 90, "tilt_deg": 0},
    ]
    assert payload["rejection_reason"] is None


def test_object_dimensions_serializes_rejection() -> None:
    result = ObjectDimensions(
        object_id="box-0042",
        timestamp_s=1.25,
        frame_id="ur10e_attachment_site",
        dimensions_m=None,
        dimensions_snapped_m=None,
        uncertainty_m=None,
        pose=None,
        views_used=(ViewDescriptor("SCAN_YAW_0", 0, 0),),
        confidence=0.0,
        valid=False,
        rejection_reason=RejectionReason.MISSING_BACKGROUND,
    )
    payload = result.to_dict()

    assert payload["rejection_reason"] == "MISSING_BACKGROUND"
    assert payload["dimensions_m"] is None


def test_rejection_reasons_cover_pipeline_failures() -> None:
    required = {
        "INSUFFICIENT_FOREGROUND",
        "FRAME_BORDER_CONTACT",
        "INSUFFICIENT_VIEWS",
        "REGISTRATION_INCONSISTENT",
        "OUT_OF_RANGE",
        "HIGH_UNCERTAINTY",
        "MOTION_TIMEOUT",
        "RENDER_FAILURE",
        "MISSING_BACKGROUND",
        "INSUFFICIENT_FACE_COVERAGE",
    }

    assert required <= {reason.value for reason in RejectionReason}


def test_held_handoff_is_ready_only_when_intact_valid_and_grasped() -> None:
    intact = ObjectDimensions(
        object_id="box-0042",
        timestamp_s=1.0,
        frame_id="ur10e_attachment_site",
        dimensions_m=Dimensions3D(0.30, 0.20, 0.15),
        dimensions_snapped_m=Dimensions3D(0.30, 0.20, 0.15),
        uncertainty_m=Extent3D(0.001, 0.002, 0.001),
        pose=None,
        views_used=(),
        confidence=0.9,
        valid=True,
        rejection_reason=None,
        condition=BoxCondition.INTACT,
        routing=RoutingHint.NORMAL,
    )
    damaged = ObjectDimensions(
        object_id="box-0042",
        timestamp_s=1.0,
        frame_id="ur10e_attachment_site",
        dimensions_m=Dimensions3D(0.30, 0.20, 0.15),
        dimensions_snapped_m=Dimensions3D(0.30, 0.20, 0.15),
        uncertainty_m=Extent3D(0.001, 0.002, 0.001),
        pose=None,
        views_used=(),
        confidence=0.9,
        valid=True,
        rejection_reason=None,
        condition=BoxCondition.DAMAGED,
        routing=RoutingHint.ERROR_ZONE,
    )

    ready = HeldBoxHandoff(dimensions=intact, held=True)
    assert ready.ready_for_com() is True
    assert HeldBoxHandoff(dimensions=intact, held=False).ready_for_com() is False
    assert HeldBoxHandoff(dimensions=damaged, held=False).ready_for_com() is False
