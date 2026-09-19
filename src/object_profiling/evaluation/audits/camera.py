from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import mujoco
import numpy as np

from ...station.camera import RGBDSensor
from ...station.controller import ScanPoseController
from ...station.environment import BoxSpec, ProfilingEnvironment
from ...station.poses import INSPECTION_POSES, SCAN_POSES
from ..checkpoint import MAXIMUM_BOX, MINIMUM_BOX, NOMINAL_BOX


MIN_VISIBLE_PIXELS = 4_000
MIN_BORDER_MARGIN_PX = 60
MIN_VISIBLE_FRACTION = 0.80


@dataclass(frozen=True)
class CameraCoverageRecord:
    object_id: str
    pose_name: str
    target_yaw_deg: int
    target_tilt_deg: int
    visible_pixels: int
    isolated_pixels: int
    visible_fraction: float
    border_margin_px: int
    bounding_box_px: tuple[int, int, int, int]
    minimum_depth_m: float
    maximum_depth_m: float
    valid: bool


def project_box_corners(environment: ProfilingEnvironment) -> np.ndarray:
    """Proyecta ground truth para auditar encuadre; no forma parte del estimador."""

    dimensions = environment.box_spec.dimensions_m.as_array()
    signs = np.asarray(
        [
            [x, y, z]
            for x in (-1.0, 1.0)
            for y in (-1.0, 1.0)
            for z in (-1.0, 1.0)
        ]
    )
    corners_local = signs * dimensions / 2.0
    box_to_world = environment.body_to_world("profiling_box")
    corners_world = (box_to_world[:3, :3] @ corners_local.T).T + box_to_world[:3, 3]

    camera_id = mujoco.mj_name2id(
        environment.model,
        mujoco.mjtObj.mjOBJ_CAMERA,
        "scan_rgbd_cam",
    )
    camera_to_world_rotation = environment.data.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
    points_camera = (camera_to_world_rotation.T @ (corners_world - environment.data.cam_xpos[camera_id]).T).T
    height = environment.config.sensor.height
    width = environment.config.sensor.width
    focal = 0.5 * height / np.tan(np.deg2rad(environment.model.cam_fovy[camera_id]) / 2.0)
    pixels = np.column_stack(
        [
            focal * points_camera[:, 0] / points_camera[:, 2] + (width - 1) / 2.0,
            focal * points_camera[:, 1] / points_camera[:, 2] + (height - 1) / 2.0,
            points_camera[:, 2],
        ]
    )
    return pixels


def _segmentation(renderer: mujoco.Renderer, environment: ProfilingEnvironment, option=None) -> np.ndarray:
    renderer.enable_segmentation_rendering()
    renderer.update_scene(environment.data, camera="scan_rgbd_cam", scene_option=option)
    return renderer.render().copy()


def _save_observation(
    directory: Path,
    record_name: str,
    rgb: np.ndarray,
    depth_m: np.ndarray,
    mask: np.ndarray,
) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(directory / f"{record_name}-rgb.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(directory / f"{record_name}-mask.png"), mask.astype(np.uint8) * 255)
    np.save(directory / f"{record_name}-depth-m.npy", depth_m)


def audit_box_camera(box_spec: BoxSpec, *, artifact_directory: Path | None = None) -> list[CameraCoverageRecord]:
    environment = ProfilingEnvironment.create(box_spec, attach_box=False)
    environment.attach_box()
    controller = ScanPoseController(environment)
    sensor = RGBDSensor(environment)
    evaluator = mujoco.Renderer(
        environment.model,
        height=environment.config.sensor.height,
        width=environment.config.sensor.width,
    )
    box_geom_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    records: list[CameraCoverageRecord] = []

    try:
        for pose in SCAN_POSES:
            pose_name, yaw_deg, tilt_deg = pose.name, pose.yaw_deg, pose.tilt_deg
            controller.move_to_qpos(pose.target_qpos(environment.config.motion))
            observation = sensor.capture(pose_name, yaw_deg=yaw_deg, tilt_deg=tilt_deg)
            segmentation = _segmentation(evaluator, environment)
            mask = segmentation[:, :, 0] == box_geom_id
            rows, columns = np.nonzero(mask)

            original_groups = environment.model.geom_group.copy()
            environment.model.geom_group[:] = 5
            environment.model.geom_group[box_geom_id] = 0
            isolated_option = mujoco.MjvOption()
            isolated_option.geomgroup[:] = 0
            isolated_option.geomgroup[0] = 1
            isolated = _segmentation(evaluator, environment, isolated_option)
            environment.model.geom_group[:] = original_groups

            visible_pixels = int(columns.size)
            isolated_pixels = int(np.count_nonzero(isolated[:, :, 0] == box_geom_id))
            if visible_pixels:
                bounding_box = (
                    int(columns.min()),
                    int(rows.min()),
                    int(columns.max()),
                    int(rows.max()),
                )
                border_margin = min(
                    bounding_box[0],
                    bounding_box[1],
                    environment.config.sensor.width - 1 - bounding_box[2],
                    environment.config.sensor.height - 1 - bounding_box[3],
                )
                box_depth = observation.depth_m[mask]
                valid_depth = box_depth[np.isfinite(box_depth) & (box_depth > 0.0)]
            else:
                bounding_box = (0, 0, 0, 0)
                border_margin = 0
                valid_depth = np.empty(0)

            visible_fraction = visible_pixels / isolated_pixels if isolated_pixels else 0.0
            valid = bool(
                visible_pixels >= MIN_VISIBLE_PIXELS
                and border_margin >= MIN_BORDER_MARGIN_PX
                and visible_fraction >= MIN_VISIBLE_FRACTION
                and valid_depth.size == visible_pixels
            )
            record = CameraCoverageRecord(
                object_id=box_spec.object_id,
                pose_name=pose_name,
                target_yaw_deg=yaw_deg,
                target_tilt_deg=tilt_deg,
                visible_pixels=visible_pixels,
                isolated_pixels=isolated_pixels,
                visible_fraction=float(visible_fraction),
                border_margin_px=int(border_margin),
                bounding_box_px=bounding_box,
                minimum_depth_m=float(valid_depth.min()) if valid_depth.size else float("nan"),
                maximum_depth_m=float(valid_depth.max()) if valid_depth.size else float("nan"),
                valid=valid,
            )
            records.append(record)
            if artifact_directory is not None:
                _save_observation(
                    artifact_directory / box_spec.object_id,
                    pose_name.lower(),
                    observation.rgb,
                    observation.depth_m,
                    mask,
                )

        controller.move_to_qpos(INSPECTION_POSES[0].target_qpos(environment.config.motion))
    finally:
        sensor.close()
        evaluator.close()
    return records


def audit_camera_suite(*, artifact_directory: Path | None = None) -> dict:
    box_specs = (MINIMUM_BOX, NOMINAL_BOX, MAXIMUM_BOX)
    records = [
        record
        for box_spec in box_specs
        for record in audit_box_camera(box_spec, artifact_directory=artifact_directory)
    ]
    camera_environment = ProfilingEnvironment.create(NOMINAL_BOX, attach_box=False)
    camera_id = mujoco.mj_name2id(camera_environment.model, mujoco.mjtObj.mjOBJ_CAMERA, "scan_rgbd_cam")
    return {
        "schema_version": 1,
        "checkpoint": "fixed_rgbd_camera_coverage",
        "valid": all(record.valid for record in records),
        "camera": {
            "name": "scan_rgbd_cam",
            "position_world_m": [float(value) for value in camera_environment.data.cam_xpos[camera_id]],
            "target_world_m": list(camera_environment.config.sensor.scan_center_world_m),
            "resolution_px": [camera_environment.config.sensor.width, camera_environment.config.sensor.height],
            "vertical_fov_deg": float(camera_environment.model.cam_fovy[camera_id]),
        },
        "thresholds": {
            "minimum_visible_pixels": MIN_VISIBLE_PIXELS,
            "minimum_border_margin_px": MIN_BORDER_MARGIN_PX,
            "minimum_visible_fraction": MIN_VISIBLE_FRACTION,
        },
        "records": [asdict(record) for record in records],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Audita la cobertura de la camara RGB-D fija en las tres poses.")
    parser.add_argument("--artifacts", type=Path, help="Directorio opcional para guardar RGB, profundidad y mascara GT.")
    parser.add_argument("--output", type=Path, help="Ruta opcional para el informe JSON.")
    args = parser.parse_args()

    report = audit_camera_suite(artifact_directory=args.artifacts)
    payload = json.dumps(report, indent=2, sort_keys=True)
    print(payload)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
