from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from ...config import AppConfig
from ...measure.perception import ScanView, observation_to_scan_view
from ...measure.registration import (
    TOOL_FRAME_ID,
    fuse_scan_views,
    view_extent_disagreement_m,
    view_plane_residuals_m,
)
from ...station.camera import RGBDSensor
from ...station.environment import BoxSpec, ProfilingEnvironment
from ...station.poses import SCAN_POSES
from ...station.scanning import run_fixed_scan
from ..checkpoint import MAXIMUM_BOX, MINIMUM_BOX, NOMINAL_BOX
from ..metrics import RegistrationMetrics, box_to_tool, evaluate_registration


MAX_P95_SURFACE_DISTANCE_M = 0.003
MAX_VIEW_PLANE_RESIDUAL_M = 0.004


@dataclass(frozen=True)
class RegistrationRecord:
    object_id: str
    frame_id: str
    views_used: tuple[str, ...]
    fused_points: int
    per_view_registration: tuple[RegistrationMetrics, ...]
    fused_registration: RegistrationMetrics
    view_plane_residuals_m: tuple[float, ...]
    view_extent_disagreement_m: tuple[float, float, float]
    valid: bool


def audit_box_registration(
    box_spec: BoxSpec,
    *,
    config: AppConfig | None = None,
    artifact_directory: Path | None = None,
) -> RegistrationRecord:
    config = config or AppConfig()
    environment = ProfilingEnvironment.create(box_spec, config, attach_box=False)
    sensor = RGBDSensor(environment)
    views: list[ScanView] = []
    truth_transforms: list[np.ndarray] = []
    rejections: list[str] = []

    def on_capture(pose, observation, backgrounds) -> None:
        # Se registra antes de segmentar para que la pose real corresponda a la
        # captura, aunque no participe en la solucion.
        truth_transforms.append(box_to_tool(environment))
        view, reason = observation_to_scan_view(observation, backgrounds.depth_for(pose.name), config)
        if view is None:
            rejections.append(f"{pose.name}:{reason.value if reason else 'UNKNOWN'}")
            truth_transforms.pop()
            return
        views.append(view)

    try:
        run_fixed_scan(environment, sensor, on_capture=on_capture)
    finally:
        sensor.close()

    dimensions = box_spec.dimensions_m.as_array()
    cloud = fuse_scan_views(views)
    per_view = tuple(
        evaluate_registration(cloud.points_for_view(index), truth_transforms[index], dimensions)
        for index in range(cloud.view_count)
    )
    # La caja es rigida respecto al terminal, asi que la pose de la primera
    # captura sirve de referencia comun para la nube fusionada.
    fused = evaluate_registration(cloud.points_m, truth_transforms[0], dimensions)
    residuals = view_plane_residuals_m(cloud, config.estimator.percentile_low, config.estimator.percentile_high)
    disagreement = view_extent_disagreement_m(cloud)

    record = RegistrationRecord(
        object_id=box_spec.object_id,
        frame_id=cloud.frame_id,
        views_used=tuple(view.pose_name for view in cloud.views),
        fused_points=int(cloud.points_m.shape[0]),
        per_view_registration=per_view,
        fused_registration=fused,
        view_plane_residuals_m=tuple(float(value) for value in residuals),
        view_extent_disagreement_m=tuple(float(value) for value in disagreement),
        valid=bool(
            not rejections
            and cloud.view_count == len(SCAN_POSES)
            and fused.p95_surface_distance_m <= MAX_P95_SURFACE_DISTANCE_M
            and float(np.max(residuals)) <= MAX_VIEW_PLANE_RESIDUAL_M
        ),
    )

    # Los casos fallidos conservan siempre su nube; los validos solo si se pide.
    if artifact_directory is not None or not record.valid:
        directory = artifact_directory or Path("artifacts/registration-audit")
        directory.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            directory / f"{box_spec.object_id}-fused-cloud.npz",
            points_tool_m=cloud.points_m,
            view_index=cloud.view_index,
            frame_id=np.asarray(TOOL_FRAME_ID),
            pose_names=np.asarray([view.pose_name for view in cloud.views]),
        )
    return record


def audit_registration_suite(*, artifact_directory: Path | None = None) -> dict:
    records = [
        audit_box_registration(box_spec, artifact_directory=artifact_directory)
        for box_spec in (MINIMUM_BOX, NOMINAL_BOX, MAXIMUM_BOX)
    ]
    return {
        "schema_version": 1,
        "checkpoint": "multiview_registration",
        "frame_id": TOOL_FRAME_ID,
        "valid": all(record.valid for record in records),
        "thresholds": {
            "maximum_p95_surface_distance_m": MAX_P95_SURFACE_DISTANCE_M,
            "maximum_view_plane_residual_m": MAX_VIEW_PLANE_RESIDUAL_M,
        },
        "worst_fused_p95_surface_distance_m": max(
            record.fused_registration.p95_surface_distance_m for record in records
        ),
        "worst_fused_mean_surface_distance_m": max(
            record.fused_registration.mean_surface_distance_m for record in records
        ),
        "worst_view_plane_residual_m": max(
            max(record.view_plane_residuals_m) for record in records
        ),
        "records": [asdict(record) for record in records],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mide el registro de las tres vistas en el marco del terminal."
    )
    parser.add_argument("--artifacts", type=Path, help="Directorio opcional para las nubes fusionadas.")
    parser.add_argument("--output", type=Path, help="Ruta opcional para el informe JSON.")
    args = parser.parse_args()

    report = audit_registration_suite(artifact_directory=args.artifacts)
    payload = json.dumps(report, indent=2, sort_keys=True)
    print(payload)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
