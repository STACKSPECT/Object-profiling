from __future__ import annotations

import dataclasses
from pathlib import Path

import numpy as np

from object_profiling.config import AppConfig, EstimatorConfig
from object_profiling.presentation.demo import HEADER_HEIGHT, TILE, compose_panels, run_demo
from object_profiling.station.pipeline import profile_seed

SEED = 42
PACKAGE = Path(__file__).resolve().parents[1] / "src" / "object_profiling"


def test_headless_demo_measures_and_renders_the_mosaic() -> None:
    result, panels = run_demo(SEED, visual=False, speed=1.0)

    assert result.dimensions.valid is True
    assert panels.dtype == np.uint8
    assert panels.shape[1] == TILE[0] * 3
    # Cabecera, tres filas de vistas y la fila de nube y resultado.
    assert panels.shape[0] == HEADER_HEIGHT + 4 * (TILE[1] + HEADER_HEIGHT)
    assert panels.any()


def test_the_displayed_mask_can_only_be_the_observable_one() -> None:
    """La demo no tiene acceso al renderizador de mascaras ground truth."""

    source = (PACKAGE / "presentation" / "demo.py").read_text(encoding="utf-8")

    assert "GroundTruthRenderer" not in source
    assert "enable_segmentation_rendering" not in source
    assert "box_geom" not in source
    # Lo unico que consulta de la escena son las dimensiones reales, y solo para
    # el panel de evaluacion.
    assert source.count("box_spec") == 1
    assert "medido_inicial" in source


def test_a_rejected_profile_still_renders() -> None:
    config = AppConfig(estimator=dataclasses.replace(EstimatorConfig(), max_uncertainty_m=1e-9))
    result = profile_seed(SEED, config=config)

    panels = compose_panels(result, np.asarray([0.30, 0.20, 0.15]), "REJECTED")

    assert result.dimensions.valid is False
    assert panels.shape[1] == TILE[0] * 3
    assert panels.any()


def test_the_mosaic_holds_one_tile_per_pose_of_the_fixed_sequence() -> None:
    result, panels = run_demo(SEED, visual=False, speed=1.0)

    assert len(result.views) == 3
    assert [view.pose_name for view in result.views] == [
        "SCAN_YAW_0",
        "SCAN_YAW_90",
        "SCAN_TILT_35",
    ]
    assert panels.shape[1] == TILE[0] * len(result.views)


def test_visual_and_headless_share_the_measurement_path() -> None:
    """La demo no puede tener su propia logica de medicion."""

    source = (PACKAGE / "presentation" / "demo.py").read_text(encoding="utf-8")

    assert source.count("profile(") >= 2
    assert "estimate_cuboid" not in source
    assert "segment_foreground" not in source
    assert "fuse_scan_views" not in source
