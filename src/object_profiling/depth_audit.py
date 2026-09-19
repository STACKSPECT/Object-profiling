from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import mujoco
import numpy as np

from .checkpoint import MAXIMUM_BOX, MINIMUM_BOX, NOMINAL_BOX
from .contracts import CameraObservation
from .controller import ScanPoseController
from .environment import BoxSpec, ProfilingEnvironment
from .poses import SCAN_POSES
from .sensors import RGBDSensor


PIXEL_STRIDE = 4
# Anillo de la mascara ground truth, solo para esta auditoria. Separa el
# interior de las caras del borde donde el rasterizado miente. No es el camino
# de la solucion: el estimador conserva la silueta (EXP-006).
SILHOUETTE_EROSION_PX = 2


@dataclass(frozen=True)
class DepthAccuracyRecord:
    object_id: str
    pose_name: str
    quantization_step_m: float
    # Interior de la caja: mascara GT erosionada, solo para auditar el buffer.
    interior_pixels: int
    interior_mean_absolute_error_m: float
    interior_p95_absolute_error_m: float
    interior_maximum_absolute_error_m: float
    interior_signed_bias_m: float
    # Borde de silueta: el anillo que esta auditoria descarta. Ahi el rasterizado
    # y el rayo del centro del pixel pueden caer en superficies distintas.
    silhouette_pixels: int
    silhouette_p95_absolute_error_m: float
    silhouette_maximum_absolute_error_m: float


def _analytic_ray_distances(
    environment: ProfilingEnvironment,
    observation: CameraObservation,
    rows: np.ndarray,
    columns: np.ndarray,
) -> np.ndarray:
    """Distancia geometrica exacta por pixel mediante trazado de rayos.

    `mj_ray` resuelve la interseccion contra la geometria real de la escena, sin
    pasar por el buffer de profundidad, asi que sirve de referencia para medir
    el error del render. Es una herramienta de auditoria.
    """

    intrinsics = observation.intrinsics
    directions_camera = np.column_stack(
        [
            (columns.astype(np.float64) - intrinsics.cx) / intrinsics.fx,
            (rows.astype(np.float64) - intrinsics.cy) / intrinsics.fy,
            np.ones(rows.size),
        ]
    )
    rotation = observation.camera_to_world[:3, :3]
    origin = observation.camera_to_world[:3, 3]
    directions_world = directions_camera @ rotation.T
    norms = np.linalg.norm(directions_world, axis=1)
    directions_world /= norms[:, None]

    geom_id = np.zeros(1, dtype=np.int32)
    distances = np.empty(rows.size, dtype=np.float64)
    for index in range(rows.size):
        distance = mujoco.mj_ray(
            environment.model,
            environment.data,
            origin,
            directions_world[index],
            None,
            1,
            -1,
            geom_id,
        )
        distances[index] = distance if geom_id[0] >= 0 else np.nan
    # `mj_ray` mide a lo largo del rayo; la profundidad del render es la
    # coordenada z en el marco de la camara.
    return distances / norms


def _ground_truth_box_mask(renderer: mujoco.Renderer, environment: ProfilingEnvironment) -> np.ndarray:
    """Mascara ground truth de la caja, solo para auditoria."""

    box_geom_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    renderer.enable_segmentation_rendering()
    renderer.update_scene(environment.data, camera="scan_rgbd_cam")
    segmentation = renderer.render().copy()
    renderer.disable_segmentation_rendering()
    return segmentation[:, :, 0] == box_geom_id


def _quantization_step_m(depths: np.ndarray) -> float:
    """Salto minimo representable observado entre valores de profundidad."""

    unique = np.unique(depths)
    if unique.size < 2:
        return float("nan")
    gaps = np.diff(unique)
    return float(np.median(gaps))


def audit_depth_accuracy(box_spec: BoxSpec) -> list[DepthAccuracyRecord]:
    environment = ProfilingEnvironment.create(box_spec, attach_box=False)
    environment.attach_box()
    controller = ScanPoseController(environment)
    sensor = RGBDSensor(environment)
    evaluator = mujoco.Renderer(
        environment.model,
        height=environment.config.sensor.height,
        width=environment.config.sensor.width,
    )
    records: list[DepthAccuracyRecord] = []

    try:
        for pose in SCAN_POSES:
            controller.move_to_qpos(pose.target_qpos(environment.config.motion))
            observation = sensor.capture(pose.name, yaw_deg=pose.yaw_deg, tilt_deg=pose.tilt_deg)
            mask = _ground_truth_box_mask(evaluator, environment)
            interior = cv2.erode(mask.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)

            rows, columns = np.nonzero(mask)
            keep = slice(None, None, PIXEL_STRIDE)
            rows, columns = rows[keep], columns[keep]
            rendered = observation.depth_m[rows, columns]
            reference = _analytic_ray_distances(environment, observation, rows, columns)
            usable = np.isfinite(reference)
            rows, columns = rows[usable], columns[usable]
            rendered, reference = rendered[usable], reference[usable]
            absolute = np.abs(rendered - reference)
            signed = rendered - reference
            on_interior = interior[rows, columns]

            records.append(
                DepthAccuracyRecord(
                    object_id=box_spec.object_id,
                    pose_name=pose.name,
                    quantization_step_m=_quantization_step_m(rendered),
                    interior_pixels=int(np.count_nonzero(on_interior)),
                    interior_mean_absolute_error_m=float(np.mean(absolute[on_interior])),
                    interior_p95_absolute_error_m=float(np.percentile(absolute[on_interior], 95)),
                    interior_maximum_absolute_error_m=float(np.max(absolute[on_interior])),
                    interior_signed_bias_m=float(np.mean(signed[on_interior])),
                    silhouette_pixels=int(np.count_nonzero(~on_interior)),
                    silhouette_p95_absolute_error_m=float(np.percentile(absolute[~on_interior], 95))
                    if np.any(~on_interior)
                    else 0.0,
                    silhouette_maximum_absolute_error_m=float(np.max(absolute[~on_interior]))
                    if np.any(~on_interior)
                    else 0.0,
                )
            )
    finally:
        sensor.close()
        evaluator.close()
    return records


def audit_depth_suite() -> dict:
    environment = ProfilingEnvironment.create(NOMINAL_BOX, attach_box=False)
    records = [
        record
        for box_spec in (MINIMUM_BOX, NOMINAL_BOX, MAXIMUM_BOX)
        for record in audit_depth_accuracy(box_spec)
    ]
    return {
        "schema_version": 1,
        "checkpoint": "render_depth_accuracy",
        "render": {
            "model_extent_m": float(environment.model.stat.extent),
            "map_znear": float(environment.model.vis.map.znear),
            "map_zfar": float(environment.model.vis.map.zfar),
            "near_plane_m": float(environment.model.vis.map.znear * environment.model.stat.extent),
            "far_plane_m": float(environment.model.vis.map.zfar * environment.model.stat.extent),
        },
        "protocol": {
            "reference": "mj_ray analytic intersection",
            "region": "ground truth box mask, audit only",
            "pixel_stride": PIXEL_STRIDE,
            "silhouette_erosion_px": SILHOUETTE_EROSION_PX,
        },
        "worst_interior_p95_absolute_error_m": max(record.interior_p95_absolute_error_m for record in records),
        "worst_interior_signed_bias_m": max(record.interior_signed_bias_m for record in records),
        "worst_silhouette_maximum_absolute_error_m": max(
            record.silhouette_maximum_absolute_error_m for record in records
        ),
        "worst_quantization_step_m": max(record.quantization_step_m for record in records),
        "records": [asdict(record) for record in records],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mide el error de la profundidad renderizada frente al trazado de rayos analitico."
    )
    parser.add_argument("--output", type=Path, help="Ruta opcional para el informe JSON.")
    args = parser.parse_args()

    report = audit_depth_suite()
    payload = json.dumps(report, indent=2, sort_keys=True)
    print(payload)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
