from __future__ import annotations

import mujoco
import numpy as np

from .contracts import CameraIntrinsics, CameraObservation, RejectionReason
from .environment import ProfilingEnvironment


class RenderError(RuntimeError):
    """El render no produjo una observacion utilizable."""

    def __init__(self, detail: str):
        super().__init__(detail)
        self.reason = RejectionReason.RENDER_FAILURE


class RGBDSensor:
    def __init__(self, environment: ProfilingEnvironment, camera_name: str = "scan_rgbd_cam"):
        self.environment = environment
        self.camera_name = camera_name
        sensor = environment.config.sensor
        self.renderer = mujoco.Renderer(environment.model, height=sensor.height, width=sensor.width)

    def close(self) -> None:
        self.renderer.close()

    def capture(self, pose_name: str, *, yaw_deg: int, tilt_deg: int) -> CameraObservation:
        env = self.environment
        try:
            self.renderer.disable_depth_rendering()
            self.renderer.update_scene(env.data, camera=self.camera_name)
            rgb = self.renderer.render().copy()
            self.renderer.enable_depth_rendering()
            self.renderer.update_scene(env.data, camera=self.camera_name)
            depth = self.renderer.render().copy().astype(np.float64)
            self.renderer.disable_depth_rendering()
        except Exception as error:  # el contexto grafico puede fallar en runtime
            raise RenderError(f"{pose_name}: {error}") from error
        if not np.any(np.isfinite(depth) & (depth > 0.0)):
            raise RenderError(f"{pose_name}: profundidad sin ningun valor valido")

        camera_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)
        fovy = float(env.model.cam_fovy[camera_id])
        height, width = depth.shape
        fy = 0.5 * height / np.tan(np.deg2rad(fovy) / 2.0)
        intrinsics = CameraIntrinsics(width, height, fy, fy, (width - 1) / 2.0, (height - 1) / 2.0)

        camera_rotation_mujoco = env.data.cam_xmat[camera_id].reshape(3, 3)
        transform = np.eye(4)
        transform[:3, :3] = camera_rotation_mujoco @ np.diag([1.0, -1.0, -1.0])
        transform[:3, 3] = env.data.cam_xpos[camera_id]
        return CameraObservation(
            timestamp_s=float(env.data.time),
            pose_name=pose_name,
            target_yaw_deg=yaw_deg,
            target_tilt_deg=tilt_deg,
            rgb=rgb,
            depth_m=depth,
            intrinsics=intrinsics,
            camera_to_world=transform,
            tool_to_world=env.tool_to_world(),
        )


def lateral_pitch_m(observation: CameraObservation, mask: np.ndarray) -> float:
    """Tamano lateral de un pixel a la distancia observada.

    Es el suelo con el que puede situarse un borde de silueta, y por tanto un
    limite fisico de la incertidumbre dimensional.
    """

    depths = observation.depth_m[mask]
    valid = depths[np.isfinite(depths) & (depths > 0.0)]
    if valid.size == 0:
        return float("inf")
    return float(np.median(valid) / observation.intrinsics.fx)


def backproject_depth(observation: CameraObservation, mask: np.ndarray) -> np.ndarray:
    rows, cols = np.nonzero(mask)
    depths = observation.depth_m[rows, cols]
    valid = np.isfinite(depths) & (depths > 0.0)
    rows, cols, depths = rows[valid], cols[valid], depths[valid]
    intrinsics = observation.intrinsics
    x = (cols.astype(np.float64) - intrinsics.cx) * depths / intrinsics.fx
    y = (rows.astype(np.float64) - intrinsics.cy) * depths / intrinsics.fy
    points_camera = np.column_stack([x, y, depths, np.ones_like(depths)])
    return (observation.camera_to_world @ points_camera.T).T[:, :3]
