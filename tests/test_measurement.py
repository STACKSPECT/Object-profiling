from __future__ import annotations

from object_profiling.config import AppConfig, DamageConfig
from object_profiling.contracts import (
    CameraIntrinsics,
    CameraObservation,
    RejectionReason,
)
from object_profiling.measure.background import BackgroundSet, PoseBackground
from object_profiling.measure.measurement import measure, rejected_measurement

import numpy as np


def _empty_observation(pose_name: str) -> CameraObservation:
    height, width = 8, 8
    return CameraObservation(
        timestamp_s=0.0,
        pose_name=pose_name,
        target_yaw_deg=0,
        target_tilt_deg=0,
        rgb=np.zeros((height, width, 3), dtype=np.uint8),
        depth_m=np.ones((height, width), dtype=np.float64),
        intrinsics=CameraIntrinsics(width, height, 100.0, 100.0, 3.5, 3.5),
        camera_to_world=np.eye(4),
        tool_to_world=np.eye(4),
    )


def test_measure_rejects_a_pose_without_background() -> None:
    result = measure(
        (_empty_observation("SCAN_YAW_0"),),
        BackgroundSet(()),
        AppConfig(),
        object_id="box-0001",
    )

    assert result.dimensions.valid is False
    assert result.dimensions.rejection_reason is RejectionReason.MISSING_BACKGROUND
    assert result.dimensions.routing.value == "NORMAL"


def test_measure_rejects_empty_foreground() -> None:
    background = BackgroundSet(
        (
            PoseBackground(
                pose_name="SCAN_YAW_0",
                depth_m=np.ones((8, 8), dtype=np.float64),
                joint_positions_rad=np.zeros(6),
            ),
        )
    )
    result = measure(
        (_empty_observation("SCAN_YAW_0"),),
        background,
        AppConfig(),
        object_id="box-0001",
    )

    assert result.dimensions.valid is False
    assert result.dimensions.rejection_reason is RejectionReason.INSUFFICIENT_FOREGROUND


def test_rejected_measurement_is_unknown_and_not_routed_to_error() -> None:
    result = rejected_measurement("box-0001", 1.0, RejectionReason.MOTION_TIMEOUT)

    assert result.dimensions.valid is False
    assert result.dimensions.condition.value == "UNKNOWN"
    assert result.dimensions.routing.value == "NORMAL"


def test_damage_config_exposes_stacking_thresholds() -> None:
    config = DamageConfig()

    assert config.absolute_floor_m == 0.003
    assert config.relative_threshold == 0.05
    assert not hasattr(config, "rate")
