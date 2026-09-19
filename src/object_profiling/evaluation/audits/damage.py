"""Auditoria de generacion de cajas danadas: mosaico RGB y desviacion de profundidad."""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

import cv2
import mujoco
import numpy as np

from ...config import AppConfig
from ...station.controller import ScanPoseController
from ...station.damage import DamageKind, DamageSpec
from ...station.environment import ProfilingEnvironment
from ...station.poses import SCAN_POSES
from ..checkpoint import NOMINAL_BOX


def _rgb(environment: ProfilingEnvironment) -> np.ndarray:
    renderer = mujoco.Renderer(environment.model, height=480, width=640)
    renderer.update_scene(environment.data, camera="scan_rgbd_cam")
    image = renderer.render().copy()
    renderer.close()
    return image


def _depth(environment: ProfilingEnvironment) -> np.ndarray:
    renderer = mujoco.Renderer(environment.model, height=480, width=640)
    renderer.enable_depth_rendering()
    renderer.update_scene(environment.data, camera="scan_rgbd_cam")
    depth = renderer.render().copy()
    renderer.close()
    return depth


def _box_with(kind: DamageKind, severity_m: float, location: str, radius_m: float = 0.0):
    return replace(
        NOMINAL_BOX,
        object_id=f"{kind.value}-{int(severity_m * 1000)}",
        damage=DamageSpec(kind=kind, severity_m=severity_m, location=location, radius_m=radius_m),
    )


def run_damage_audit(*, artifacts: Path | None = None) -> dict:
    environment = ProfilingEnvironment.create(NOMINAL_BOX, AppConfig(), attach_box=True)
    controller = ScanPoseController(environment)
    scan_qpos = SCAN_POSES[0].target_qpos(environment.config.motion)

    def present() -> None:
        controller.move_to_qpos(scan_qpos)

    present()
    baseline = _depth(environment)
    cases = [
        ("INTACT", NOMINAL_BOX),
        ("CRUSHED_15", _box_with(DamageKind.CRUSHED_CORNER, 0.015, "corner:+x+y+z")),
        ("CRUSHED_40", _box_with(DamageKind.CRUSHED_CORNER, 0.040, "corner:+x+y+z")),
        ("DENT_10", _box_with(DamageKind.DENTED_FACE, 0.010, "face:+x", 0.05)),
        ("DENT_25", _box_with(DamageKind.DENTED_FACE, 0.025, "face:+x", 0.05)),
        ("BUCKLE_20", _box_with(DamageKind.BUCKLED_PANEL, 0.020, "face:+x")),
    ]
    records = []
    tiles = []
    for label, spec in cases:
        environment.load_box(spec, attach_box=True)
        present()
        rgb = _rgb(environment)
        depth = _depth(environment)
        diff = np.abs(depth - baseline)
        both_near = (depth < 2.0) & (baseline < 2.0)
        surface = both_near & (diff > 5e-4)
        peak = float(diff[surface].max()) if surface.any() else 0.0
        records.append(
            {
                "label": label,
                "kind": spec.damage.kind.value,
                "severity_m": spec.damage.severity_m,
                "location": spec.damage.location,
                "changed_pixels": int(surface.sum()),
                "silhouette_pixels": int(((depth < 2.0) != (baseline < 2.0)).sum()),
                "peak_deviation_m": peak,
            }
        )
        captioned = cv2.putText(
            cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
            f"{label}  {peak * 1000:.1f} mm",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (20, 20, 20),
            2,
            cv2.LINE_AA,
        )
        tiles.append(captioned)
    mosaic = np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:])])
    if artifacts is not None:
        artifacts.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(artifacts / "damage-mosaic.png"), mosaic)
    return {"schema_version": 1, "checkpoint": "damage_generation", "records": records}


def main() -> int:
    parser = argparse.ArgumentParser(description="Renderiza cajas danadas y mide la desviacion de profundidad.")
    parser.add_argument("--artifacts", type=Path, default=Path("artifacts/damage-audit"))
    parser.add_argument("--output", type=Path, help="JSON de salida.")
    args = parser.parse_args()
    report = run_damage_audit(artifacts=args.artifacts)
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
