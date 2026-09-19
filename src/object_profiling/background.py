from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass

import numpy as np

from .contracts import RejectionReason
from .controller import ScanPoseController
from .environment import ProfilingEnvironment
from .poses import SCAN_POSES, ScanPose
from .sensors import RGBDSensor


class MissingBackgroundError(KeyError):
    """No existe fondo calibrado para una pose de escaneo."""

    def __init__(self, pose_name: str):
        super().__init__(pose_name)
        self.pose_name = pose_name
        self.reason = RejectionReason.MISSING_BACKGROUND


@dataclass(frozen=True)
class PoseBackground:
    """Profundidad de la estacion vacia en una pose concreta.

    El brazo y el terminal cambian de sitio entre poses, asi que un unico
    fondo no sirve para las tres. La asociacion es por `pose_name`.
    """

    pose_name: str
    depth_m: np.ndarray
    joint_positions_rad: np.ndarray


@dataclass(frozen=True)
class BackgroundSet:
    backgrounds: tuple[PoseBackground, ...]

    def depth_for(self, pose_name: str) -> np.ndarray:
        for background in self.backgrounds:
            if background.pose_name == pose_name:
                return background.depth_m
        raise MissingBackgroundError(pose_name)

    def joint_positions_for(self, pose_name: str) -> np.ndarray:
        for background in self.backgrounds:
            if background.pose_name == pose_name:
                return background.joint_positions_rad
        raise MissingBackgroundError(pose_name)

    @property
    def pose_names(self) -> tuple[str, ...]:
        return tuple(background.pose_name for background in self.backgrounds)

    def covers(self, pose_names: Iterable[str]) -> bool:
        available = set(self.pose_names)
        return all(name in available for name in pose_names)


def capture_pose_backgrounds(
    environment: ProfilingEnvironment,
    sensor: RGBDSensor,
    *,
    poses: tuple[ScanPose, ...] = SCAN_POSES,
    joint_positions_rad: Mapping[str, np.ndarray] | None = None,
) -> BackgroundSet:
    """Recorre las poses con la estacion vacia y registra su profundidad.

    Se ejecuta como pasada previa, antes de activar la succion, para que el
    ciclo de medicion no tenga que retirar y volver a soldar la caja entre
    capturas. Si se aportan configuraciones articulares medidas, el brazo se
    coloca exactamente en ellas en lugar de volver a estabilizarse: elimina la
    diferencia de carga entre la estacion vacia y la caja suspendida.
    """

    if environment.box_visible:
        raise ValueError("los fondos deben capturarse con la estacion vacia")

    controller = ScanPoseController(environment)
    backgrounds: list[PoseBackground] = []
    for pose in poses:
        recorded = None if joint_positions_rad is None else joint_positions_rad.get(pose.name)
        if recorded is None:
            controller.move_to_qpos(pose.target_qpos(environment.config.motion))
        else:
            environment.set_joint_positions(np.asarray(recorded, dtype=np.float64))
        environment.set_box_visible(False)
        observation = sensor.capture(pose.name, yaw_deg=pose.yaw_deg, tilt_deg=pose.tilt_deg)
        backgrounds.append(
            PoseBackground(
                pose_name=pose.name,
                depth_m=observation.depth_m.copy(),
                joint_positions_rad=environment.data.qpos[:6].copy(),
            )
        )
    return BackgroundSet(tuple(backgrounds))
