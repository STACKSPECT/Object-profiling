from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from object_profiling.config import AppConfig, EstimatorConfig, SensorConfig
from object_profiling.contracts import OBJECT_DIMENSIONS_SCHEMA_VERSION, RejectionReason
from object_profiling.station.environment import ProfilingEnvironment, generate_box_spec
from object_profiling.station.poses import SCAN_POSES
from object_profiling.station.pipeline import profile, profile_seed, profile_session
from object_profiling.measure.registration import TOOL_FRAME_ID
from object_profiling.station.scanning import run_fixed_scan
from object_profiling.station.camera import RGBDSensor

SEED = 42


def _with_estimator(**changes) -> AppConfig:
    return AppConfig(estimator=dataclasses.replace(EstimatorConfig(), **changes))


def _with_sensor(**changes) -> AppConfig:
    return AppConfig(sensor=dataclasses.replace(SensorConfig(), **changes))


@pytest.fixture(scope="module")
def nominal_profile():
    return profile_seed(SEED)


def test_the_fixed_cycle_produces_a_valid_profile(nominal_profile) -> None:
    result = nominal_profile.dimensions

    assert result.valid is True
    assert result.rejection_reason is None
    assert result.schema_version == OBJECT_DIMENSIONS_SCHEMA_VERSION
    assert result.frame_id == TOOL_FRAME_ID
    assert result.object_id == f"box-{SEED:04d}"
    assert result.dimensions_m is not None
    assert result.uncertainty_m is not None
    assert 0.0 <= result.confidence <= 1.0
    assert result.condition.value == "INTACT"
    assert result.routing.value == "NORMAL"
    assert result.damage is None


def test_every_pose_of_the_fixed_sequence_is_used(nominal_profile) -> None:
    assert [view.pose_name for view in nominal_profile.dimensions.views_used] == [
        pose.name for pose in SCAN_POSES
    ]
    assert nominal_profile.rejected_views == ()
    assert len(nominal_profile.views) == 2


def test_the_inspection_yaw_is_not_fed_to_the_estimator() -> None:
    environment = ProfilingEnvironment.for_seed(SEED, attach_box=False)
    sensor = RGBDSensor(environment)
    try:
        cycle = run_fixed_scan(environment, sensor)
    finally:
        sensor.close()

    assert [observation.pose_name for observation in cycle.observations] == [
        "SCAN_YAW_0",
        "SCAN_YAW_90",
    ]
    assert [observation.pose_name for observation in cycle.inspection_observations] == [
        "SCAN_YAW_180",
    ]


def test_the_profile_is_close_to_the_ground_truth(nominal_profile) -> None:
    """La comparacion es evaluacion; el estimador no la ve."""

    truth = generate_box_spec(SEED, AppConfig()).dimensions_m.as_array()
    estimated = nominal_profile.dimensions.dimensions_m.as_array()
    snapped = nominal_profile.dimensions.dimensions_snapped_m.as_array()
    uncertainty = nominal_profile.dimensions.uncertainty_m.as_array()

    assert np.all(np.abs(estimated - truth) < 0.005)
    assert np.all(np.abs(estimated - truth) <= uncertainty)
    assert snapped == pytest.approx(truth)


def test_the_serialised_contract_is_json_ready(nominal_profile) -> None:
    payload = nominal_profile.dimensions.to_dict()

    assert payload["frame_id"] == TOOL_FRAME_ID
    assert payload["rejection_reason"] is None
    assert payload["views_used"][1] == {
        "pose_name": "SCAN_YAW_90",
        "yaw_deg": 90,
        "tilt_deg": 0,
    }
    assert set(payload) == {
        "schema_version",
        "object_id",
        "timestamp_s",
        "frame_id",
        "dimensions_m",
        "dimensions_snapped_m",
        "uncertainty_m",
        "pose",
        "views_used",
        "confidence",
        "valid",
        "rejection_reason",
        "condition",
        "routing",
        "damage",
    }


def test_repeating_a_seed_reproduces_the_profile() -> None:
    first = profile_seed(SEED).dimensions.to_dict()
    second = profile_seed(SEED).dimensions.to_dict()

    assert first == second


def test_latency_and_cycle_time_are_recorded(nominal_profile) -> None:
    assert nominal_profile.perception_latency_s > 0.0
    assert nominal_profile.cycle_duration_s > nominal_profile.perception_latency_s


@pytest.mark.parametrize(
    ("config", "expected"),
    [
        (
            _with_sensor(min_component_pixels=10**7),
            RejectionReason.INSUFFICIENT_FOREGROUND,
        ),
        (
            _with_estimator(minimum_face_support_points=10**6),
            RejectionReason.INSUFFICIENT_FACE_COVERAGE,
        ),
        (
            _with_estimator(max_uncertainty_m=1e-9),
            RejectionReason.HIGH_UNCERTAINTY,
        ),
        (
            _with_estimator(max_view_plane_residual_m=1e-9),
            RejectionReason.REGISTRATION_INCONSISTENT,
        ),
        (
            _with_estimator(minimum_points=10**9),
            RejectionReason.INSUFFICIENT_FOREGROUND,
        ),
        (
            _with_estimator(minimum_views=4),
            RejectionReason.INSUFFICIENT_VIEWS,
        ),
    ],
)
def test_each_failure_surfaces_an_explicit_reason(config, expected) -> None:
    result = profile_seed(SEED, config=config).dimensions

    assert result.valid is False
    assert result.rejection_reason is expected
    assert result.dimensions_m is None
    assert result.uncertainty_m is None
    assert result.confidence == 0.0
    # Un rechazo sigue declarando el marco y el objeto.
    assert result.frame_id == TOOL_FRAME_ID
    assert result.object_id == f"box-{SEED:04d}"


def test_out_of_range_dimensions_are_rejected() -> None:
    """La caja del episodio es real; el rango declarado es el que no la admite."""

    narrow = AppConfig(box_range=dataclasses.replace(AppConfig().box_range, height_m=(0.08, 0.10)))
    environment = ProfilingEnvironment.for_seed(SEED, AppConfig(), attach_box=False)

    result = profile(environment, seed=SEED, config=narrow).dimensions

    assert result.valid is False
    assert result.rejection_reason is RejectionReason.OUT_OF_RANGE


def test_the_measurement_cycle_never_hides_the_box_between_captures() -> None:
    """Los fondos se toman en una pasada previa, no retirando la caja entre poses.

    Si el ciclo ocultase la caja entre capturas reconstruiria el weld de succion
    a mitad de ciclo y la deriva auditada en EXP-001 dejaria de ser comparable.
    """

    environment = ProfilingEnvironment.for_seed(SEED, attach_box=False)
    sensor = RGBDSensor(environment)
    calls: list[tuple[str, bool]] = []
    original = environment.set_box_visible

    def record(visible: bool) -> None:
        calls.append(("set_box_visible", visible))
        original(visible)

    environment.set_box_visible = record
    try:
        run_fixed_scan(
            environment,
            sensor,
            on_capture=lambda *_args: calls.append(("capture", environment.box_visible)),
        )
    finally:
        sensor.close()

    hides_before_first_capture = calls.index(("capture", True))
    assert all(name == "set_box_visible" for name, _ in calls[:hides_before_first_capture])
    assert [value for name, value in calls if name == "capture"] == [True, True]
    assert environment.box_attached


def test_a_session_profiles_n_boxes_on_the_same_station() -> None:
    environment = ProfilingEnvironment.for_seed(SEED, attach_box=False)
    model = environment.model
    states: list[str] = []

    results = profile_session(SEED, 2, environment=environment, on_state=states.append)

    assert [result.dimensions.object_id for result in results] == ["box-0042", "box-0043"]
    assert all(result.dimensions.valid for result in results)
    assert environment.model is model
    assert environment.object_id == "box-0043"
    assert states.count("CALIBRATE_BACKGROUND") == 1
    assert states.index("CLEAR_BOX") > states.index("BOX_1_OF_2")
    assert states.index("CLEAR_BOX") < states.index("BOX_2_OF_2")
    assert "BOX_2_OF_2" in states


def test_a_session_rejects_a_non_positive_count() -> None:
    with pytest.raises(ValueError, match="count must be at least 1"):
        profile_session(SEED, 0)
