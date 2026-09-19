"""Ablacion de poses de escaneo: calidad de L/W/H y snap a 5 mm.

Captura las tres vistas una vez por episodio y mide subconjuntos. El tiempo de
ciclo de dos poses se mide aparte, ejecutando la trayectoria real recortada.
"""

from __future__ import annotations

import argparse
import json
import time
from collections import Counter
from dataclasses import replace
from pathlib import Path

import numpy as np

from ..config import AppConfig, EstimatorConfig
from ..measure.measurement import measure
from ..station.camera import RGBDSensor
from ..station.environment import ProfilingEnvironment, generate_box_spec
from ..station.poses import SCAN_POSES, SCAN_TILT_35, SCAN_YAW_90, ScanPose
from ..station.scanning import run_fixed_scan
from .checkpoint import MAXIMUM_BOX, MINIMUM_BOX, NOMINAL_BOX

QUALITY_SUBSETS: dict[str, tuple[str, ...]] = {
    "Y0": ("SCAN_YAW_0",),
    "Y90": ("SCAN_YAW_90",),
    "T35": ("SCAN_TILT_35",),
    "Y0+Y90": ("SCAN_YAW_0", "SCAN_YAW_90"),
    "Y0+T35": ("SCAN_YAW_0", "SCAN_TILT_35"),
    "Y90+T35": ("SCAN_YAW_90", "SCAN_TILT_35"),
    "all3": ("SCAN_YAW_0", "SCAN_YAW_90", "SCAN_TILT_35"),
}

CYCLE_SUBSETS: dict[str, tuple[ScanPose, ...]] = {
    "Y0+Y90": SCAN_POSES,
    "Y90+T35": (SCAN_YAW_90, SCAN_TILT_35),
    "all3": (*SCAN_POSES, SCAN_TILT_35),
}

CONTRAST_CASES = (
    ("minima", MINIMUM_BOX),
    ("nominal", NOMINAL_BOX),
    ("maxima", MAXIMUM_BOX),
    ("seed42", generate_box_spec(42, AppConfig())),
    ("seed72", generate_box_spec(72, AppConfig())),
    ("seed731", generate_box_spec(731, AppConfig())),
)


def _errors_mm(estimated, truth_m: np.ndarray) -> np.ndarray | None:
    if estimated is None:
        return None
    return (estimated.as_array() - truth_m) * 1000.0


def _snap_matches(snapped, truth_m: np.ndarray) -> bool | None:
    if snapped is None:
        return None
    return bool(np.allclose(snapped.as_array(), truth_m, atol=1e-9))


def evaluate_episode(environment: ProfilingEnvironment, config: AppConfig) -> dict:
    """Captura las dos poses de produccion y el tilt, y mide subconjuntos."""

    sensor = RGBDSensor(environment)
    try:
        cycle = run_fixed_scan(
            environment, sensor, poses=(*SCAN_POSES, SCAN_TILT_35), finish_poses=()
        )
    finally:
        sensor.close()
    truth_m = environment.box_spec.dimensions_m.as_array()
    by_name = {observation.pose_name: observation for observation in cycle.observations}
    subsets: dict[str, dict] = {}
    for name, pose_names in QUALITY_SUBSETS.items():
        result = measure(
            [by_name[pose_name] for pose_name in pose_names],
            cycle.backgrounds,
            config,
            object_id=environment.object_id,
        )
        dimensions = result.dimensions
        error = _errors_mm(dimensions.dimensions_m, truth_m)
        subsets[name] = {
            "valid": dimensions.valid,
            "rejection_reason": None if dimensions.rejection_reason is None else dimensions.rejection_reason.value,
            "error_mm": None if error is None else [float(value) for value in error],
            "snap_matches": _snap_matches(dimensions.dimensions_snapped_m, truth_m),
            "weakest_support": result.estimate.coverage.weakest_support if result.estimate else 0,
        }
    return {
        "object_id": environment.object_id,
        "truth_m": [float(value) for value in truth_m],
        "subsets": subsets,
    }


def _summarize(rows: list[dict], subset: str) -> dict:
    selected = [row["subsets"][subset] for row in rows]
    valid = [item for item in selected if item["valid"] and item["error_mm"] is not None]
    errors = np.abs(np.asarray([item["error_mm"] for item in valid])) if valid else np.empty((0, 3))
    snaps = [item["snap_matches"] for item in selected if item["snap_matches"] is not None]
    return {
        "episodes": len(selected),
        "valid": len(valid),
        "rejected": len(selected) - len(valid),
        "snap_matches": int(sum(bool(value) for value in snaps)),
        "snap_rate": (sum(bool(value) for value in snaps) / len(snaps)) if snaps else None,
        "mae_mm": {
            axis: float(np.mean(errors[:, index])) if valid else None
            for index, axis in enumerate(("length", "width", "height"))
        },
        "p95_mm": {
            axis: float(np.percentile(errors[:, index], 95)) if valid else None
            for index, axis in enumerate(("length", "width", "height"))
        },
        "max_mm": {
            axis: float(np.max(errors[:, index])) if valid else None
            for index, axis in enumerate(("length", "width", "height"))
        },
        "worst_absolute_mm": float(np.max(errors)) if valid else None,
        "rejection_reasons": dict(
            Counter(item["rejection_reason"] for item in selected if item["rejection_reason"])
        ),
    }


def time_cycle(environment: ProfilingEnvironment, poses: tuple[ScanPose, ...]) -> dict:
    sensor = RGBDSensor(environment)
    started = time.perf_counter()
    simulation_started = float(environment.data.time)
    try:
        run_fixed_scan(environment, sensor, poses=poses, finish_poses=())
    finally:
        sensor.close()
    return {
        "wall_clock_s": time.perf_counter() - started,
        "simulated_s": float(environment.data.time) - simulation_started,
    }


def run_ablation(
    *,
    seeds: range,
    cycle_seeds: range,
    config: AppConfig | None = None,
    progress: bool = False,
) -> dict:
    config = config or AppConfig(estimator=replace(EstimatorConfig(), minimum_views=1))
    contrast_rows = []
    for label, spec in CONTRAST_CASES:
        environment = ProfilingEnvironment.create(spec, config, attach_box=False)
        row = evaluate_episode(environment, config)
        row["label"] = label
        contrast_rows.append(row)
        if progress:
            print(f"[ablation] contrast {label}")

    seed_rows = []
    for index, seed in enumerate(seeds):
        environment = ProfilingEnvironment.for_seed(seed, config, attach_box=False)
        row = evaluate_episode(environment, config)
        row["seed"] = seed
        seed_rows.append(row)
        if progress and (index + 1) % 10 == 0:
            print(f"[ablation] {index + 1}/{len(seeds)} seeds")

    cycle_rows: dict[str, list[dict]] = {name: [] for name in CYCLE_SUBSETS}
    for seed in cycle_seeds:
        for name, poses in CYCLE_SUBSETS.items():
            environment = ProfilingEnvironment.for_seed(seed, config, attach_box=False)
            timed = time_cycle(environment, poses)
            timed["seed"] = seed
            cycle_rows[name].append(timed)
        if progress:
            print(f"[ablation] cycle seed {seed}")

    def cycle_summary(name: str) -> dict:
        walls = np.asarray([row["wall_clock_s"] for row in cycle_rows[name]])
        sims = np.asarray([row["simulated_s"] for row in cycle_rows[name]])
        return {
            "n": len(walls),
            "wall_p50_s": float(np.median(walls)),
            "simulated_p50_s": float(np.median(sims)),
        }

    return {
        "schema_version": 1,
        "checkpoint": "pose_ablation",
        "seeds": [seeds.start, seeds.stop],
        "cycle_seeds": [cycle_seeds.start, cycle_seeds.stop],
        "contrast": {subset: _summarize(contrast_rows, subset) for subset in QUALITY_SUBSETS},
        "variable": {subset: _summarize(seed_rows, subset) for subset in QUALITY_SUBSETS},
        "cycle_time": {name: cycle_summary(name) for name in CYCLE_SUBSETS},
        "contrast_rows": contrast_rows,
        "seed_rows": seed_rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compara subconjuntos de las tres poses de escaneo contra el catalogo de 5 mm."
    )
    parser.add_argument("--start", type=int, default=1000)
    parser.add_argument("--count", type=int, default=50)
    parser.add_argument("--cycle-count", type=int, default=8)
    parser.add_argument("--output", type=Path, default=Path("results/pose-ablation.json"))
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    report = run_ablation(
        seeds=range(args.start, args.start + args.count),
        cycle_seeds=range(args.start, args.start + args.cycle_count),
        progress=not args.quiet,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if not args.quiet:
        print(json.dumps({key: report[key] for key in ("contrast", "variable", "cycle_time")}, indent=2))
        print(f"[ablation] escrito {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
