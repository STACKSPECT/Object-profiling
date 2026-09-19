"""Auditoria de senales de inspeccion frente al DamageSpec de escena."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from ...config import AppConfig
from ...station.environment import ProfilingEnvironment, generate_box_spec
from ...station.pipeline import profile


def run_inspection_audit(*, start: int, count: int, damage_rate: float, config: AppConfig | None = None) -> dict:
    from dataclasses import replace

    config = config or AppConfig()
    config = replace(config, damage=replace(config.damage, rate=damage_rate))
    records = []
    for seed in range(start, start + count):
        spec = generate_box_spec(seed, config)
        environment = ProfilingEnvironment.create(spec, config, attach_box=False)
        result = profile(environment, seed=seed, config=config)
        inspection = result.measurement.inspection
        records.append(
            {
                "seed": seed,
                "truth_kind": spec.damage.kind.value,
                "truth_severity_m": spec.damage.severity_m,
                "truth_location": spec.damage.location,
                "condition": result.dimensions.condition.value,
                "routing": result.dimensions.routing.value,
                "predicted_kind": None if result.dimensions.damage is None else result.dimensions.damage.kind.value,
                "predicted_severity_m": None if result.dimensions.damage is None else result.dimensions.damage.severity_m,
                "valid": result.dimensions.valid,
                "inspection": None
                if inspection is None
                else {
                    "residual_p95_m": inspection.residual_p95_m,
                    "max_inward_m": inspection.max_inward_m,
                    "max_edge_p95_m": inspection.max_edge_p95_m,
                    "weakest_corner_support": inspection.weakest_corner_support,
                    "inward_fraction": inspection.inward_fraction,
                    "inward_location": inspection.inward_location,
                    "corner_support": list(inspection.corner_support),
                },
            }
        )
    intact = [record for record in records if record["truth_kind"] == "INTACT"]
    damaged = [record for record in records if record["truth_kind"] != "INTACT"]
    false_positives = [record for record in intact if record["condition"] == "DAMAGED"]
    detections = [record for record in damaged if record["condition"] == "DAMAGED"]
    return {
        "schema_version": 1,
        "checkpoint": "damage_inspection",
        "damage_rate": damage_rate,
        "summary": {
            "intact": len(intact),
            "damaged": len(damaged),
            "false_positives": len(false_positives),
            "detections": len(detections),
            "misses": len(damaged) - len(detections),
            "by_truth_kind": {
                kind: {
                    "count": sum(1 for record in records if record["truth_kind"] == kind),
                    "detected": sum(
                        1
                        for record in records
                        if record["truth_kind"] == kind and record["condition"] == "DAMAGED"
                    ),
                }
                for kind in ("INTACT", "CRUSHED_CORNER", "DENTED_FACE", "BUCKLED_PANEL")
            },
        },
        "records": records,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Mide senales de dano sobre una banda de seeds.")
    parser.add_argument("--start", type=int, default=8000)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--damage-rate", type=float, default=0.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = run_inspection_audit(start=args.start, count=args.count, damage_rate=args.damage_rate)
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
