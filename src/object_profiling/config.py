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
    # Absorbe el asentamiento del terminal entre la estacion vacia del fondo y
    # la caja suspendida, que EXP-003 midio por debajo de 0,1 mm.
    foreground_margin_m: float = 0.004
    # EXP-002 midio 5.010 px visibles en el peor caso, la caja minima. El umbral
    # de colocacion de camara de la auditoria son 4.000 px; aqui solo hace falta
    # descartar restos que no puedan ser la caja, y la mascara util se erosiona.
    min_component_pixels: int = 2_000
    border_margin_px: int = 3
    # EXP-006: la silueta es la unica fuente de los extremos que ninguna cara
    # observada define, asi que erosionarla los borraba. Se conserva completa.
    mask_erosion_px: int = 0
    scan_center_world_m: tuple[float, float, float] = (-0.174, 0.735, 0.650)
    # Holgura del volumen de recorte sobre el tamano maximo de caja declarado.
    tool_volume_margin_m: float = 0.04
    # Separacion entre el marco del terminal y el plano de contacto de las copas,
    # donde se apoya la cara superior de la caja. Es geometria del terminal.
    tool_to_box_offset_m: float = 0.089
    # Holgura por encima de ese plano. Las copas ocupan z entre 0,065 y 0,089 en
    # el marco del terminal: un margen generoso los mete en la nube y estira la
    # altura hasta 30 mm.
    cup_plane_margin_m: float = 0.002


@dataclass(frozen=True)
class EstimatorConfig:
    # EXP-006: recortar medio punto porcentual por extremo costaba hasta 11 mm
    # de anchura, porque los extremos que solo define la silueta tienen poca
    # densidad. Con 0,05 el error queda por debajo del milimetro.
    percentile_low: float = 0.05
    percentile_high: float = 99.95
    bootstrap_samples: int = 24
    bootstrap_point_cap: int = 12_000
    minimum_points: int = 800
    minimum_views: int = 3
    # Cobertura: cuantos puntos deben apoyar cada extremo dentro de la loncha.
    # El extremo peor medido sobre el rango de cajas aporta 121 puntos.
    coverage_slab_m: float = 0.002
    minimum_face_support_points: int = 50
    # Residuo p95 por vista medido sobre el rango de cajas: 0,805 mm en el peor
    # caso. El umbral deja margen y detecta un desplazamiento de 10 mm.
    max_view_plane_residual_m: float = 0.003
    max_uncertainty_m: float = 0.012
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
