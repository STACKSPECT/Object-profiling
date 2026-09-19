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


def test_the_contrast_cases_really_cover_contrasting_shapes() -> None:
    cases = contrast_cases(CONFIG)

    assert len(cases) == 6
    shapes = {case.label: case.spec(CONFIG).dimensions_m for case in cases}
    assert shapes["minima del rango"] == MINIMUM_BOX.dimensions_m
    assert shapes["maxima del rango"] == MAXIMUM_BOX.dimensions_m

    elongated = shapes["alargada  L/W 3.00"]
    assert elongated.length / elongated.width > 2.6

    cubic = shapes["casi cubica  L/W 1.00"]
    assert cubic.length / cubic.width < 1.06
    assert 0.94 < cubic.height / cubic.width < 1.06


def test_every_case_is_reproducible_from_its_definition() -> None:
    first, second = contrast_cases(CONFIG), contrast_cases(CONFIG)

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
    return run_showcase(visual=False, announce=False)


def test_the_showcase_measures_every_case(showcase) -> None:
    outcomes, _sheet = showcase

    assert len(outcomes) == 6
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

    assert report["cases"] == 6
    assert report["valid_profiles"] == 6
    errors = np.abs(np.asarray([outcome.error_mm for outcome in outcomes]))
    for index, axis in enumerate(("length", "width", "height")):
        assert report["mae_mm"][axis] == pytest.approx(float(np.mean(errors[:, index])))
    assert report["worst_absolute_error_mm"] == pytest.approx(float(np.max(errors)))
    assert len(report["results"]) == 6
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


def test_the_showcase_reuses_one_viewer() -> None:
    """MuJoCo solo admite un visor por proceso: abrir el segundo falla."""

    source = (PACKAGE / "presentation" / "showcase.py").read_text(encoding="utf-8")

    assert source.count("launch_passive") == 1
