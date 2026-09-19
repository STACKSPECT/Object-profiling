from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCENE_PATH = PROJECT_ROOT / "assets" / "universal_robots_ur10e" / "profiling_scene.xml"


@dataclass(frozen=True)
class BoxRange:
    length_m: tuple[float, float] = (0.15, 0.40)
    width_m: tuple[float, float] = (0.12, 0.30)
    height_m: tuple[float, float] = (0.08, 0.25)
    mass_kg: tuple[float, float] = (0.5, 5.0)


@dataclass(frozen=True)
class SensorConfig:
    width: int = 640
    height: int = 480
    foreground_margin_m: float = 0.004
    min_component_pixels: int = 500
    border_margin_px: int = 3
    scan_center_world_m: tuple[float, float, float] = (-0.174, 0.691, 0.535)
    scan_half_extent_m: tuple[float, float, float] = (0.34, 0.34, 0.30)


@dataclass(frozen=True)
class EstimatorConfig:
    percentile_low: float = 0.5
    percentile_high: float = 99.5
    bootstrap_samples: int = 24
    bootstrap_point_cap: int = 12_000
    minimum_points: int = 800
    max_uncertainty_m: float = 0.012
    max_view_height_delta_m: float = 0.015
    cuboid_residual_scale_m: float = 0.012


@dataclass(frozen=True)
class MotionConfig:
    home_qpos: tuple[float, ...] = (-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0)
    # Solucion IK validada para elevar el TCP 0.12 m manteniendo su orientacion.
    lift_qpos: tuple[float, ...] = (
        -1.57080019,
        -1.54995221,
        1.33798071,
        -1.35883691,
        -1.57079971,
        -0.00000019,
    )
    # IK offline: inclinacion fija de 35 grados alrededor del eje X local del
    # terminal, manteniendo la posicion del TCP de la pose elevada.
    tilt_35_qpos: tuple[float, ...] = (
        -1.57080030,
        -1.62733782,
        1.56764639,
        -2.12198114,
        -1.57079896,
        -0.00000221,
    )
    trajectory_duration_s: float = 0.65
    settle_duration_s: float = 0.25
    joint_error_rad: float = 0.025
    joint_velocity_rad_s: float = 0.04
    timeout_s: float = 3.0

    def target(self, angle_deg: int, *, lifted: bool = True) -> np.ndarray:
        base = self.lift_qpos if lifted else self.home_qpos
        target = np.asarray(base, dtype=np.float64).copy()
        target[-1] += np.deg2rad(float(angle_deg))
        return target

    def tilt_target(self) -> np.ndarray:
        return np.asarray(self.tilt_35_qpos, dtype=np.float64).copy()


@dataclass(frozen=True)
class AppConfig:
    box_range: BoxRange = field(default_factory=BoxRange)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    estimator: EstimatorConfig = field(default_factory=EstimatorConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    required_views_deg: tuple[int, ...] = (0, 90)
    fallback_view_deg: int = 180
