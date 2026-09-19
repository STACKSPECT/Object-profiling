from __future__ import annotations

import argparse
import json
import time
from collections import Counter
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .config import AppConfig
from .contracts import Extent3D, EvaluationRecord
from .environment import ProfilingEnvironment
from .evaluation import box_to_tool, evaluate_registration
from .profiling_pipeline import profile

# Criterios originales del plan. Se reportan tal cual, se cumplan o no.
TARGET_VALID_RATE = 0.98
TARGET_MAE_M = 0.005
TARGET_P95_M = 0.010


def _absolute_error(estimated: np.ndarray, truth: np.ndarray) -> Extent3D:
    error = np.abs(estimated - truth)
    return Extent3D(float(error[0]), float(error[1]), float(error[2]))


def evaluate_seed(seed: int, config: AppConfig | None = None) -> tuple[EvaluationRecord, dict]:
    """Mide una seed y la compara contra el ground truth del episodio."""

    config = config or AppConfig()
    environment = ProfilingEnvironment.for_seed(seed, config, attach_box=False)
    truth = environment.box_spec.dimensions_m

    started = time.perf_counter()
    result = profile(environment, seed=seed, config=config)
    cycle_duration = time.perf_counter() - started

    truth_array = truth.as_array()
    estimated = result.dimensions.dimensions_m
    record = EvaluationRecord(
        seed=seed,
        ground_truth_m=truth,
        prediction=result.dimensions,
        absolute_error_m=_absolute_error(estimated.as_array(), truth_array) if estimated else None,
        perception_latency_s=result.perception_latency_s,
    )

    registration = None
    if result.cloud is not None:
        metrics = evaluate_registration(result.cloud.points_m, box_to_tool(environment), truth_array)
        registration = asdict(metrics)
    diagnostics = {
        "cycle_duration_s": cycle_duration,
        "mass_kg": environment.box_spec.mass_kg,
        "registration": registration,
        "rejected_views": [list(entry) for entry in result.rejected_views],
        "weakest_face_support": result.estimate.coverage.weakest_support if result.estimate else 0,
    }
    return record, diagnostics


def run_benchmark(
    *,
    seeds: range,
    config: AppConfig | None = None,
    determinism_samples: int = 5,
    progress: bool = False,
) -> dict:
    config = config or AppConfig()
    records: list[EvaluationRecord] = []
    diagnostics: list[dict] = []
    for index, seed in enumerate(seeds):
        record, diagnostic = evaluate_seed(seed, config)
        records.append(record)
        diagnostics.append(diagnostic)
        if progress and (index + 1) % 10 == 0:
            print(f"[benchmark] {index + 1}/{len(seeds)} seeds")

    valid = [record for record in records if record.prediction.valid]
    errors = np.asarray([record.absolute_error_m.as_array() for record in valid]) if valid else np.empty((0, 3))
    uncertainties = (
        np.asarray([record.prediction.uncertainty_m.as_array() for record in valid])
        if valid
        else np.empty((0, 3))
    )
    latencies = np.asarray([record.perception_latency_s for record in records])
    cycles = np.asarray([diagnostic["cycle_duration_s"] for diagnostic in diagnostics])
    registration_p95 = np.asarray(
        [
            diagnostic["registration"]["p95_surface_distance_m"]
            for diagnostic in diagnostics
            if diagnostic["registration"] is not None
        ]
    )

    axes = ("length", "width", "height")
    # Sin perfiles validos no hay error que resumir. Se reporta el vacio en lugar
    # de inventar un numero.
    per_axis = {
        axis: {
            "mae_m": float(np.mean(errors[:, index])) if valid else None,
            "rmse_m": float(np.sqrt(np.mean(np.square(errors[:, index])))) if valid else None,
            "p95_m": float(np.percentile(errors[:, index], 95)) if valid else None,
            "max_m": float(np.max(errors[:, index])) if valid else None,
            "mean_uncertainty_m": float(np.mean(uncertainties[:, index])) if valid else None,
            "uncertainty_coverage": float(np.mean(errors[:, index] <= uncertainties[:, index]))
            if valid
            else None,
        }
        for index, axis in enumerate(axes)
    }

    determinism = _check_determinism(seeds, config, determinism_samples)
    valid_rate = len(valid) / len(records)
    meets_targets = bool(
        valid
        and valid_rate >= TARGET_VALID_RATE
        and all(per_axis[axis]["mae_m"] <= TARGET_MAE_M for axis in axes)
        and all(per_axis[axis]["p95_m"] <= TARGET_P95_M for axis in axes)
        and determinism["reproducible"]
    )

    return {
        "schema_version": 1,
        "checkpoint": "variable_dimension_benchmark",
        "seeds": {"start": seeds.start, "stop": seeds.stop, "count": len(records)},
        "targets": {
            "minimum_valid_rate": TARGET_VALID_RATE,
            "maximum_mae_m": TARGET_MAE_M,
            "maximum_p95_m": TARGET_P95_M,
        },
        "meets_targets": meets_targets,
        "valid_profile_rate": valid_rate,
        "valid_profiles": len(valid),
        "per_axis": per_axis,
        "worst_absolute_error_m": float(np.max(errors)) if valid else None,
        "rejections_by_reason": dict(
            Counter(
                record.prediction.rejection_reason.value
                for record in records
                if record.prediction.rejection_reason is not None
            )
        ),
        "latency_s": {
            "p50": float(np.percentile(latencies, 50)),
            "p95": float(np.percentile(latencies, 95)),
        },
        "cycle_duration_s": {
            "p50": float(np.percentile(cycles, 50)),
            "p95": float(np.percentile(cycles, 95)),
        },
        "registration_p95_surface_distance_m": {
            "mean": float(np.mean(registration_p95)) if registration_p95.size else None,
            "max": float(np.max(registration_p95)) if registration_p95.size else None,
        },
        "mean_confidence": float(np.mean([record.prediction.confidence for record in valid]))
        if valid
        else 0.0,
        "determinism": determinism,
        "records": [record.to_dict() for record in records],
        "diagnostics": diagnostics,
    }


def _check_determinism(seeds: range, config: AppConfig, samples: int) -> dict:
    """Repite un subconjunto de seeds y compara la salida completa."""

    checked = list(seeds)[:samples]
    mismatches = []
    for seed in checked:
        first, _ = evaluate_seed(seed, config)
        second, _ = evaluate_seed(seed, config)
        if first.prediction.to_dict() != second.prediction.to_dict():
            mismatches.append(seed)
    return {
        "seeds_checked": checked,
        "mismatched_seeds": mismatches,
        "reproducible": not mismatches,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Benchmark de dimensiones variables sobre un rango de seeds."
    )
    parser.add_argument("--start", type=int, default=1000)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--determinism-samples", type=int, default=5)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--output", type=Path, help="Ruta opcional para el informe JSON completo.")
    args = parser.parse_args()

    report = run_benchmark(
        seeds=range(args.start, args.start + args.count),
        determinism_samples=args.determinism_samples,
        progress=not args.quiet,
    )
    summary = {key: value for key, value in report.items() if key not in {"records", "diagnostics"}}
    print(json.dumps(summary, indent=2, sort_keys=True))
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if report["meets_targets"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
