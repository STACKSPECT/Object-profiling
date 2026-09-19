from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from object_profiling.benchmark import (
    TARGET_MAE_M,
    TARGET_P95_M,
    TARGET_VALID_RATE,
    evaluate_seed,
    run_benchmark,
)
from object_profiling.config import AppConfig, EstimatorConfig
from object_profiling.contracts import RejectionReason

SEEDS = range(3000, 3006)


@pytest.fixture(scope="module")
def report():
    return run_benchmark(seeds=SEEDS, determinism_samples=2)


def test_targets_match_the_plan() -> None:
    """Los umbrales son los del plan y no se relajan para que salga verde."""

    assert TARGET_VALID_RATE == 0.98
    assert TARGET_MAE_M == 0.005
    assert TARGET_P95_M == 0.010


def test_a_single_seed_is_evaluated_against_its_ground_truth() -> None:
    record, diagnostics = evaluate_seed(SEEDS.start)

    assert record.seed == SEEDS.start
    assert record.prediction.valid is True
    assert record.absolute_error_m is not None
    expected = np.abs(
        record.prediction.dimensions_m.as_array() - record.ground_truth_m.as_array()
    )
    assert record.absolute_error_m.as_array() == pytest.approx(expected)
    assert diagnostics["registration"]["p95_surface_distance_m"] < 0.001
    assert diagnostics["cycle_duration_s"] > 0.0


def test_the_benchmark_reports_every_required_metric(report) -> None:
    assert report["seeds"]["count"] == len(SEEDS)
    assert set(report["per_axis"]) == {"length", "width", "height"}
    for axis in report["per_axis"].values():
        assert set(axis) == {
            "mae_m",
            "rmse_m",
            "p95_m",
            "max_m",
            "mean_uncertainty_m",
            "uncertainty_coverage",
        }
    assert set(report["latency_s"]) == {"p50", "p95"}
    assert set(report["cycle_duration_s"]) == {"p50", "p95"}
    assert "rejections_by_reason" in report
    assert len(report["records"]) == len(SEEDS)


def test_variable_dimensions_are_measured_within_the_targets(report) -> None:
    assert report["valid_profile_rate"] >= TARGET_VALID_RATE
    for axis in report["per_axis"].values():
        assert axis["mae_m"] <= TARGET_MAE_M
        assert axis["p95_m"] <= TARGET_P95_M
    assert report["meets_targets"] is True


def test_uncertainty_covers_the_measured_error(report) -> None:
    for axis in report["per_axis"].values():
        assert axis["uncertainty_coverage"] == 1.0


def test_repeating_a_seed_gives_an_identical_profile(report) -> None:
    assert report["determinism"]["reproducible"] is True
    assert report["determinism"]["mismatched_seeds"] == []
    assert len(report["determinism"]["seeds_checked"]) == 2


def test_a_failing_configuration_is_reported_and_not_hidden() -> None:
    """Un rechazo cuenta como fallo del benchmark, no como medida correcta."""

    config = AppConfig(estimator=dataclasses.replace(EstimatorConfig(), max_uncertainty_m=1e-9))

    failing = run_benchmark(seeds=range(3000, 3003), config=config, determinism_samples=1)

    assert failing["valid_profile_rate"] == 0.0
    assert failing["valid_profiles"] == 0
    assert failing["meets_targets"] is False
    assert failing["rejections_by_reason"] == {RejectionReason.HIGH_UNCERTAINTY.value: 3}
    assert failing["worst_absolute_error_m"] is None
    # Sin perfiles validos no se resume ningun error.
    assert all(axis["mae_m"] is None for axis in failing["per_axis"].values())
