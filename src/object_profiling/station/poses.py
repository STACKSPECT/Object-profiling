from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import MotionConfig
from ..contracts import ViewDescriptor


@dataclass(frozen=True)
class ScanPose:
    """Pose de escaneo de la trayectoria fija.

    La secuencia no depende del resultado de la medicion: todas las cajas
    ejecutan las mismas poses de escaneo.
    """

    name: str
    yaw_deg: int
    tilt_deg: int

    @property
    def descriptor(self) -> ViewDescriptor:
        return ViewDescriptor(self.name, self.yaw_deg, self.tilt_deg)

    def target_qpos(self, motion: MotionConfig) -> np.ndarray:
        if self.tilt_deg:
            return motion.tilt_target()
        return motion.target(self.yaw_deg)


SCAN_YAW_0 = ScanPose("SCAN_YAW_0", 0, 0)
SCAN_YAW_90 = ScanPose("SCAN_YAW_90", 90, 0)
SCAN_YAW_180 = ScanPose("SCAN_YAW_180", 180, 0)
# Conservada para ablacion EXP-008. Fuera del ciclo de medicion.
SCAN_TILT_35 = ScanPose("SCAN_TILT_35", 0, 35)

# Vistas que alimentan measure(). La inspeccion de defectos no entra aqui.
SCAN_POSES: tuple[ScanPose, ...] = (SCAN_YAW_0, SCAN_YAW_90)
# Segundo +90° en el mismo sentido. Completa las cinco caras visibles desde
# abajo (fondo + cuatro laterales). Se fusiona solo para inspeccion.
INSPECTION_POSES: tuple[ScanPose, ...] = (SCAN_YAW_180,)
# Recorrido de estacion: medida y luego el yaw extra de inspeccion.
STATION_POSES: tuple[ScanPose, ...] = (*SCAN_POSES, *INSPECTION_POSES)


def pose_by_name(name: str) -> ScanPose:
    for pose in SCAN_POSES:
        if pose.name == name:
            return pose
    raise KeyError(name)
