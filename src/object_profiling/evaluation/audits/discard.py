"""Auditoria del descarte a la zona de error."""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

import cv2
import mujoco
import numpy as np

from ...config import AppConfig
from ...contracts import RoutingHint
from ...station.discard import box_rests_in_error_bin
from ...station.environment import ProfilingEnvironment, generate_box_spec
from ...station.pipeline import profile


def _rgb(environment: ProfilingEnvironment) -> np.ndarray:
    renderer = mujoco.Renderer(environment.model, height=480, width=640)
    renderer.update_scene(environment.data, camera="scan_rgbd_cam")
    image = renderer.render().copy()
    renderer.close()
    return image


def run_discard_audit(*, start: int, count: int, damage_rate: float, artifacts: Path | None = None) -> dict:
    config = replace(AppConfig(), damage=replace(AppConfig().damage, rate=damage_rate))
    records = []
    tiles = []
    for seed in range(start, start + count):
        spec = generate_box_spec(seed, config)
        environment = ProfilingEnvironment.create(spec, config, attach_box=False)
        result = profile(environment, seed=seed, config=config)
        in_bin = box_rests_in_error_bin(environment)
        expected_bin = result.dimensions.routing is RoutingHint.ERROR_ZONE
        records.append(
            {
                "seed": seed,
                "truth_kind": spec.damage.kind.value,
                "condition": result.dimensions.condition.value,
                "routing": result.dimensions.routing.value,
                "valid": result.dimensions.valid,
                "in_error_bin": in_bin,
                "box_attached": environment.box_attached,
                "matches_policy": in_bin is expected_bin and (not in_bin or not environment.box_attached),
            }
        )
        if artifacts is not None:
            tiles.append(
                cv2.putText(
                    cv2.cvtColor(_rgb(environment), cv2.COLOR_RGB2BGR),
                    f"{seed} {result.dimensions.routing.value}",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (20, 20, 20),
                    2,
                    cv2.LINE_AA,
                )
            )
    if artifacts is not None and tiles:
        artifacts.mkdir(parents=True, exist_ok=True)
        columns = min(3, len(tiles))
        while len(tiles) % columns:
            tiles.append(np.zeros_like(tiles[0]))
        rows = [np.hstack(tiles[index : index + columns]) for index in range(0, len(tiles), columns)]
        cv2.imwrite(str(artifacts / "discard-mosaic.png"), np.vstack(rows))
    return {
        "schema_version": 1,
        "checkpoint": "discard_error_zone",
        "damage_rate": damage_rate,
        "policy_matches": sum(1 for record in records if record["matches_policy"]),
        "count": len(records),
        "records": records,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Comprueba que las cajas danadas acaban en el contenedor.")
    parser.add_argument("--start", type=int, default=8100)
    parser.add_argument("--count", type=int, default=12)
    parser.add_argument("--damage-rate", type=float, default=0.5)
    parser.add_argument("--artifacts", type=Path, default=Path("artifacts/discard-audit"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = run_discard_audit(
        start=args.start,
        count=args.count,
        damage_rate=args.damage_rate,
        artifacts=args.artifacts,
    )
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    return 0 if report["policy_matches"] == report["count"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
