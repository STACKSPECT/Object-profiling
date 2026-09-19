"""Sondeo historico de viabilidad. La produccion de esta rama ya aplica z baja.

No volver a espejar `cam_pos` sobre la escena actual: el XML ya esta en 0,38025 m.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from ..config import AppConfig
from ..measure.measurement import measure
from ..station.camera import RGBDSensor
from ..station.controller import MotionError, ScanPoseController
from ..station.environment import ProfilingEnvironment
from ..station.poses import SCAN_YAW_0, SCAN_YAW_90, ScanPose
from ..station.scanning import run_fixed_scan
from .audits.camera import (
    MIN_BORDER_MARGIN_PX,
    MIN_VISIBLE_FRACTION,
    MIN_VISIBLE_PIXELS,
    _segmentation,
)
from .checkpoint import MAXIMUM_BOX, MINIMUM_BOX, NOMINAL_BOX

CAMERA_NAME = "scan_rgbd_cam"
SCAN_YAW_180 = ScanPose("SCAN_YAW_180", 180, 0)
PROBE_POSES = (SCAN_YAW_0, SCAN_YAW_90, SCAN_YAW_180)


def current_camera_geometry(environment: ProfilingEnvironment) -> dict:
    camera_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME)
    position = np.asarray(environment.data.cam_xpos[camera_id], dtype=float)
    target = np.asarray(environment.config.sensor.scan_center_world_m, dtype=float)
    offset = position - target
    horizontal = float(np.hypot(offset[0], offset[1]))
    elevation_deg = float(np.rad2deg(math.atan2(offset[2], horizontal)))
    proposed_z = float(target[2] - offset[2])
    return {
        "position_m": position.tolist(),
        "target_m": target.tolist(),
        "xy_m": [float(position[0]), float(position[1])],
        "z_above_target_m": float(offset[2]),
        "horizontal_distance_m": horizontal,
        "elevation_from_horizontal_deg": elevation_deg,
        "proposed_z_m": proposed_z,
        "proposed_position_m": [float(position[0]), float(position[1]), proposed_z],
        "floor_clearance_m": proposed_z,
        "looks_up_if_mirrored": proposed_z < target[2],
    }


def _set_camera_z(environment: ProfilingEnvironment, z_m: float) -> None:
    camera_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME)
    environment.model.cam_pos[camera_id][2] = z_m
    mujoco.mj_forward(environment.model, environment.data)


def _coverage_for_poses(
    box_spec,
    *,
    camera_z: float | None,
    poses: tuple[ScanPose, ...],
) -> list[dict]:
    environment = ProfilingEnvironment.create(box_spec, attach_box=False)
    if camera_z is not None:
        _set_camera_z(environment, camera_z)
    environment.attach_box()
    controller = ScanPoseController(environment)
    sensor = RGBDSensor(environment)
    evaluator = mujoco.Renderer(
        environment.model,
        height=environment.config.sensor.height,
        width=environment.config.sensor.width,
    )
    box_geom_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    records: list[dict] = []
    try:
        for pose in poses:
            controller.move_to_qpos(pose.target_qpos(environment.config.motion))
            observation = sensor.capture(pose.name, yaw_deg=pose.yaw_deg, tilt_deg=pose.tilt_deg)
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
                border_margin = 0
                valid_depth = np.empty(0)
            visible_fraction = visible_pixels / isolated_pixels if isolated_pixels else 0.0
            valid = bool(
                visible_pixels >= MIN_VISIBLE_PIXELS
                and border_margin >= MIN_BORDER_MARGIN_PX
                and visible_fraction >= MIN_VISIBLE_FRACTION
                and valid_depth.size == visible_pixels
            )
            records.append(
                {
                    "object_id": box_spec.object_id,
                    "pose_name": pose.name,
                    "visible_pixels": visible_pixels,
                    "isolated_pixels": isolated_pixels,
                    "visible_fraction": float(visible_fraction),
                    "border_margin_px": int(border_margin),
                    "minimum_depth_m": float(valid_depth.min()) if valid_depth.size else None,
                    "maximum_depth_m": float(valid_depth.max()) if valid_depth.size else None,
                    "valid_exp002": valid,
                    "finite_depth_on_box": bool(valid_depth.size == visible_pixels),
                }
            )
    finally:
        sensor.close()
        evaluator.close()
    return records


def _try_yaw(environment: ProfilingEnvironment, yaw_deg: int) -> dict:
    controller = ScanPoseController(environment)
    target = environment.config.motion.target(yaw_deg)
    try:
        controller.move_to_qpos(target)
        error = float(np.max(np.abs(environment.data.qpos[:6] - target)))
        return {"yaw_deg": yaw_deg, "reached": True, "max_joint_error_rad": error}
    except MotionError as error:
        return {"yaw_deg": yaw_deg, "reached": False, "reason": error.reason.value}


def _measure_with_camera_z(box_spec, camera_z: float | None, poses: tuple[ScanPose, ...]) -> dict:
    config = AppConfig()
    environment = ProfilingEnvironment.create(box_spec, config, attach_box=False)
    if camera_z is not None:
        _set_camera_z(environment, camera_z)
    sensor = RGBDSensor(environment)
    try:
        cycle = run_fixed_scan(environment, sensor, poses=poses)
    finally:
        sensor.close()
    result = measure(cycle.observations, cycle.backgrounds, config, object_id=environment.object_id)
    truth = box_spec.dimensions_m.as_array()
    estimated = result.dimensions.dimensions_m
    snapped = result.dimensions.dimensions_snapped_m
    error_mm = None if estimated is None else ((estimated.as_array() - truth) * 1000.0).tolist()
    return {
        "valid": result.dimensions.valid,
        "rejection_reason": None
        if result.dimensions.rejection_reason is None
        else result.dimensions.rejection_reason.value,
        "error_mm": error_mm,
        "snap_matches": None
        if snapped is None
        else bool(np.allclose(snapped.as_array(), truth, atol=1e-9)),
        "views": [pose.name for pose in poses],
    }


def run_probe() -> dict:
    reference = ProfilingEnvironment.create(NOMINAL_BOX, attach_box=False)
    geometry = current_camera_geometry(reference)
    proposed_z = geometry["proposed_z_m"]

    motion_env = ProfilingEnvironment.create(NOMINAL_BOX, attach_box=False)
    motion_env.attach_box()
    controller = ScanPoseController(motion_env)
    controller.move_to_qpos(SCAN_YAW_0.target_qpos(motion_env.config.motion))
    motion = {
        "yaw_90_from_0": _try_yaw(motion_env, 90),
        "yaw_180_from_90": _try_yaw(motion_env, 180),
    }

    coverage = {
        "current_z_y0_y90": [
            record
            for spec in (MINIMUM_BOX, NOMINAL_BOX, MAXIMUM_BOX)
            for record in _coverage_for_poses(spec, camera_z=None, poses=(SCAN_YAW_0, SCAN_YAW_90))
        ],
        "proposed_z_y0_y90_y180": [
            record
            for spec in (MINIMUM_BOX, NOMINAL_BOX, MAXIMUM_BOX)
            for record in _coverage_for_poses(
                spec, camera_z=proposed_z, poses=PROBE_POSES
            )
        ],
    }

    measurement = {
        "current_y0_y90": _measure_with_camera_z(NOMINAL_BOX, None, (SCAN_YAW_0, SCAN_YAW_90)),
        "proposed_y0_y90": _measure_with_camera_z(
            NOMINAL_BOX, proposed_z, (SCAN_YAW_0, SCAN_YAW_90)
        ),
        "proposed_y0_y90_y180": _measure_with_camera_z(NOMINAL_BOX, proposed_z, PROBE_POSES),
        "proposed_y0_y180": _measure_with_camera_z(
            NOMINAL_BOX, proposed_z, (SCAN_YAW_0, SCAN_YAW_180)
        ),
    }
    return _sanitize(
        {
            "schema_version": 1,
            "checkpoint": "low_camera_yaw180_viability",
            "geometry": geometry,
            "motion": motion,
            "coverage": coverage,
            "measurement_nominal": measurement,
        }
    )


def _sanitize(value):
    if isinstance(value, dict):
        return {key: _sanitize(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize(item) for item in value]
    if isinstance(value, np.ndarray):
        return _sanitize(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description="Sondeo de camara baja y yaw 180, sin tocar XML.")
    parser.add_argument("--output", type=Path, help="JSON opcional.")
    args = parser.parse_args()
    report = run_probe()
    payload = json.dumps(report, indent=2, sort_keys=True)
    print(payload)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
