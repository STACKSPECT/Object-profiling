from __future__ import annotations

import argparse

import numpy as np
import pytest

from object_profiling.evaluation.checkpoint import MAXIMUM_BOX, MINIMUM_BOX, NOMINAL_BOX, _positive_speed, run_checkpoint
from object_profiling.station.environment import ProfilingEnvironment


def test_box_starts_presented_but_not_attached() -> None:
    environment = ProfilingEnvironment.create(NOMINAL_BOX, attach_box=False)

    assert not environment.box_attached
    assert environment.active_cup_names() == (
        "cup_center",
        "cup_x_pos",
        "cup_x_neg",
        "cup_y_pos",
        "cup_y_neg",
    )


def test_attach_lift_and_rotate_checkpoint() -> None:
    report = run_checkpoint(seed=42)

    assert report["success"] is True
    assert report["failure_reason"] is None
    assert report["lifted_distance_m"] >= 0.10
    assert report["max_translation_drift_m"] <= 0.001
    assert report["max_rotation_drift_deg"] <= 0.5
    assert report["unexpected_box_contact_samples"] == 0
    assert [(state["target_yaw_deg"], state["target_tilt_deg"]) for state in report["states"]] == [
        (0, 0),
        (90, 0),
        (180, 0),
    ]
    assert report["states"][1]["tool_orientation_change_deg"] == pytest.approx(90.0, abs=0.5)
    assert report["states"][2]["tool_orientation_change_deg"] == pytest.approx(180.0, abs=0.5)


@pytest.mark.parametrize("box_spec", [MINIMUM_BOX, NOMINAL_BOX, MAXIMUM_BOX])
def test_fixed_motion_supports_box_range_endpoints(box_spec) -> None:
    report = run_checkpoint(seed=42, box_spec=box_spec)

    assert report["success"] is True
    assert report["unexpected_box_contact_samples"] == 0
    assert [state["name"] for state in report["states"]] == [
        "SCAN_YAW_0",
        "SCAN_YAW_90",
        "SCAN_YAW_180",
    ]


def test_checkpoint_is_deterministic() -> None:
    first = run_checkpoint(seed=42)
    second = run_checkpoint(seed=42)

    assert first == second
    assert np.isfinite(first["max_joint_error_rad"])


@pytest.mark.parametrize(("value", "expected"), [("0.5", 0.5), ("1", 1.0), ("2.5", 2.5)])
def test_positive_visual_speed(value: str, expected: float) -> None:
    assert _positive_speed(value) == expected


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_invalid_visual_speed_is_rejected(value: str) -> None:
    with pytest.raises(argparse.ArgumentTypeError):
        _positive_speed(value)
