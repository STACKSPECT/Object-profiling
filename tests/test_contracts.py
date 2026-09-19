from __future__ import annotations

import numpy as np
import pytest

from object_profiling.config import AppConfig
from object_profiling.contracts import (
    OBJECT_DIMENSIONS_SCHEMA_VERSION,
    Dimensions3D,
    Extent3D,
    ObjectDimensions,
    RejectionReason,
    ViewDescriptor,
)
from object_profiling.poses import RETURN_POSE, SCAN_POSES, pose_by_name


def test_fixed_scan_sequence_is_three_poses() -> None:
    assert [pose.name for pose in SCAN_POSES] == ["SCAN_YAW_0", "SCAN_YAW_90", "SCAN_TILT_35"]
    assert [(pose.yaw_deg, pose.tilt_deg) for pose in SCAN_POSES] == [(0, 0), (90, 0), (0, 35)]
    assert RETURN_POSE.name == "RETURNED_VERTICAL"


def test_scan_poses_resolve_to_configured_joint_targets() -> None:
    motion = AppConfig().motion

    assert SCAN_POSES[0].target_qpos(motion) == pytest.approx(motion.target(0))
    assert SCAN_POSES[1].target_qpos(motion) == pytest.approx(motion.target(90))
    assert SCAN_POSES[2].target_qpos(motion) == pytest.approx(motion.tilt_target())


def test_pose_lookup_rejects_unknown_name() -> None:
    assert pose_by_name("SCAN_TILT_35") is SCAN_POSES[2]
    with pytest.raises(KeyError):
        pose_by_name("SCAN_YAW_180")


def test_config_no_longer_exposes_adaptive_view_fields() -> None:
    config = AppConfig()

    assert not hasattr(config, "required_views_deg")
    assert not hasattr(config, "fallback_view_deg")


def test_dimensions_enforce_length_width_convention() -> None:
    dimensions = Dimensions3D(length=0.30, width=0.20, height=0.15)

    assert dimensions.as_array() == pytest.approx(np.asarray([0.30, 0.20, 0.15]))
    with pytest.raises(ValueError):
        Dimensions3D(length=0.20, width=0.30, height=0.15)


def test_extent_allows_uncertainty_smaller_on_length_than_width() -> None:
    uncertainty = Extent3D(length=0.001, width=0.004, height=0.002)

    assert uncertainty.as_array() == pytest.approx(np.asarray([0.001, 0.004, 0.002]))


def test_object_dimensions_serializes_structured_views() -> None:
    result = ObjectDimensions(
        object_id="box-0042",
        timestamp_s=1.25,
        frame_id="tool_attachment_site",
        dimensions_m=Dimensions3D(0.30, 0.20, 0.15),
        uncertainty_m=Extent3D(0.001, 0.002, 0.001),
        views_used=tuple(pose.descriptor for pose in SCAN_POSES),
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
        {"pose_name": "SCAN_TILT_35", "yaw_deg": 0, "tilt_deg": 35},
    ]
    assert payload["rejection_reason"] is None


def test_object_dimensions_serializes_rejection() -> None:
    result = ObjectDimensions(
        object_id="box-0042",
        timestamp_s=1.25,
        frame_id="tool_attachment_site",
        dimensions_m=None,
        uncertainty_m=None,
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
