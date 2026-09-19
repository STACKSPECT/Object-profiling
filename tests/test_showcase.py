from __future__ import annotations

import dataclasses
from pathlib import Path

import mujoco
import numpy as np
import pytest

from object_profiling.evaluation.checkpoint import MAXIMUM_BOX, MINIMUM_BOX
from object_profiling.config import AppConfig, EstimatorConfig
from object_profiling.station.environment import ProfilingEnvironment
from object_profiling.presentation.panels import HEADER_HEIGHT
from object_profiling.presentation.showcase import (
    CASE_TILE,
    SHEET_WIDTH,
    SHOWCASE_CASE_COUNT,
    ShowcaseCase,
    compose_sheet,
    contrast_cases,
    measure_case,
    run_showcase,
    summary,
    _table_row,
)

PACKAGE = Path(__file__).resolve().parents[1] / "src" / "object_profiling"
CONFIG = AppConfig()


def _assert_catalogue_dimensions(dimensions, config: AppConfig) -> None:
    step = config.catalogue_step_m
    values = dimensions.as_array()
    assert np.all(np.isclose(values / step, np.round(values / step)))
    assert config.box_range.length_m[0] <= dimensions.length <= config.box_range.length_m[1]
    assert config.box_range.width_m[0] <= dimensions.width <= config.box_range.width_m[1]
    assert config.box_range.height_m[0] <= dimensions.height <= config.box_range.height_m[1]
    assert dimensions.length >= dimensions.width


def test_the_showcase_draws_six_random_catalogue_boxes() -> None:
    cases = contrast_cases(base_seed=42)

    assert len(cases) == SHOWCASE_CASE_COUNT
    assert [case.seed for case in cases] == list(range(42, 42 + SHOWCASE_CASE_COUNT))
    shapes = [case.spec(CONFIG).dimensions_m for case in cases]
    for dimensions in shapes:
        _assert_catalogue_dimensions(dimensions, CONFIG)
    assert len({dimensions.as_array().tobytes() for dimensions in shapes}) > 1


def test_a_different_base_seed_draws_a_different_batch() -> None:
    first = [case.spec(CONFIG).dimensions_m.as_array() for case in contrast_cases(base_seed=42)]
    second = [case.spec(CONFIG).dimensions_m.as_array() for case in contrast_cases(base_seed=99)]

    assert not all(np.allclose(left, right) for left, right in zip(first, second))


def test_every_case_is_reproducible_from_its_definition() -> None:
    first, second = contrast_cases(base_seed=42), contrast_cases(base_seed=42)

    for left, right in zip(first, second):
        assert left.spec(CONFIG) == right.spec(CONFIG)


def test_load_box_retargets_the_same_model() -> None:
    """Reutilizar el modelo es lo que permite un unico visor por proceso."""

    environment = ProfilingEnvironment.create(MINIMUM_BOX, CONFIG, attach_box=False)
    model = environment.model
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    small = model.geom_size[geom_id].copy()

    environment.load_box(MAXIMUM_BOX)

    assert environment.model is model
    assert environment.box_spec is MAXIMUM_BOX
    assert np.all(model.geom_size[geom_id] > small)
    assert model.geom_size[geom_id] == pytest.approx(MAXIMUM_BOX.dimensions_m.as_array() / 2.0)
    assert not environment.box_attached


@pytest.fixture(scope="module")
def showcase():
    return run_showcase(visual=False, announce=False, seed=42)


def test_the_showcase_measures_every_case(showcase) -> None:
    outcomes, _sheet = showcase

    assert len(outcomes) == SHOWCASE_CASE_COUNT
    for outcome in outcomes:
        assert outcome.dimensions.valid is True, outcome.case.label
        assert outcome.error_mm is not None
        assert np.all(np.abs(outcome.error_mm) < 5.0)


def test_each_case_reports_measured_against_real(showcase) -> None:
    outcomes, _sheet = showcase

    for outcome in outcomes:
        measured = outcome.dimensions.dimensions_m.as_array()
        expected = (measured - outcome.truth_m) * 1000.0
        assert outcome.error_mm == pytest.approx(expected)
        uncertainty = outcome.dimensions.uncertainty_m.as_array() * 1000.0
        assert np.all(np.abs(outcome.error_mm) <= uncertainty)
        row = _table_row(outcome)
        assert "medido_inicial" in row
        assert "medido" in row
        assert "real" in row


def test_the_sheet_has_one_row_per_case_plus_header_and_summary(showcase) -> None:
    outcomes, sheet = showcase
    row_height = CASE_TILE[1] + HEADER_HEIGHT

    assert sheet.dtype == np.uint8
    assert sheet.shape[1] == SHEET_WIDTH
    assert sheet.shape[0] == HEADER_HEIGHT + len(outcomes) * row_height + 176
    assert sheet.any()


def test_the_summary_aggregates_the_error(showcase) -> None:
    outcomes, _sheet = showcase
    report = summary(outcomes)

    assert report["cases"] == SHOWCASE_CASE_COUNT
    assert report["valid_profiles"] == SHOWCASE_CASE_COUNT
    errors = np.abs(np.asarray([outcome.error_mm for outcome in outcomes]))
    for index, axis in enumerate(("length", "width", "height")):
        assert report["mae_mm"][axis] == pytest.approx(float(np.mean(errors[:, index])))
    assert report["worst_absolute_error_mm"] == pytest.approx(float(np.max(errors)))
    assert len(report["results"]) == SHOWCASE_CASE_COUNT
    assert report["results"][0]["prediction"]["schema_version"] == 3


def test_a_rejected_case_still_appears_on_the_sheet() -> None:
    config = AppConfig(estimator=dataclasses.replace(EstimatorConfig(), max_uncertainty_m=1e-9))
    case = ShowcaseCase("rechazo forzado", 42)
    environment = ProfilingEnvironment.create(case.spec(config), config, attach_box=False)

    outcome = measure_case(case, environment, config=config)
    sheet = compose_sheet([outcome])

    assert outcome.dimensions.valid is False
    assert outcome.error_mm is None
    assert sheet.shape[1] == SHEET_WIDTH
    assert summary([outcome])["worst_absolute_error_mm"] is None


def test_visual_and_headless_share_the_measurement_path() -> None:
    source = (PACKAGE / "presentation" / "showcase.py").read_text(encoding="utf-8")

    # Una sola llamada a measure_case, usada por ambos modos.
    assert source.count("measure_case(case, environment") == 1
    assert "estimate_cuboid" not in source
    assert "segment_foreground" not in source
    assert "launch_passive" in source


def test_ground_truth_stays_out_of_the_estimator() -> None:
    """Las aristas reales alimentan el panel, no `measure()` ni `profile()`."""

    showcase = (PACKAGE / "presentation" / "showcase.py").read_text(encoding="utf-8")
    pipeline = (PACKAGE / "station" / "pipeline.py").read_text(encoding="utf-8")
    measurement = (PACKAGE / "measure" / "measurement.py").read_text(encoding="utf-8")

    assert "box_spec.dimensions" not in pipeline
    assert "environment.box_spec" not in pipeline.split("def profile", 1)[1]
    assert "box_spec" not in measurement
    assert "ground_truth" not in measurement
    assert "generate_box_spec" not in measurement
    assert "truth_m = environment.box_spec.dimensions_m.as_array()" in showcase
    assert "profile(environment, seed=case.seed, config=config, on_state=on_state, on_step=on_step)" in showcase
    assert "MINIMUM_BOX" not in showcase
    assert "MAXIMUM_BOX" not in showcase


def test_the_showcase_reuses_one_viewer() -> None:
    """MuJoCo solo admite un visor por proceso: abrir el segundo falla."""

    source = (PACKAGE / "presentation" / "showcase.py").read_text(encoding="utf-8")

    assert source.count("launch_passive") == 1
