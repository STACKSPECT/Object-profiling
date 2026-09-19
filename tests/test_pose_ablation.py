from __future__ import annotations

from dataclasses import replace

from object_profiling.config import AppConfig, EstimatorConfig
from object_profiling.evaluation.checkpoint import NOMINAL_BOX
from object_profiling.evaluation.pose_ablation import evaluate_episode
from object_profiling.station.environment import ProfilingEnvironment


def test_two_vertical_views_recover_the_catalogue_sku() -> None:
    """YAW_0 + YAW_90 deben bastar para el snap a 5 mm en la caja nominal."""

    config = AppConfig(estimator=replace(EstimatorConfig(), minimum_views=1))
    environment = ProfilingEnvironment.create(NOMINAL_BOX, config, attach_box=False)
    row = evaluate_episode(environment, config)

    two = row["subsets"]["Y0+Y90"]
    three = row["subsets"]["all3"]
    assert two["valid"] is True
    assert three["valid"] is True
    assert two["snap_matches"] is True
    assert three["snap_matches"] is True
    assert max(abs(value) for value in two["error_mm"]) < 2.5
