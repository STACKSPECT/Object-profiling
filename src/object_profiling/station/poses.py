from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import MotionConfig
from ..contracts import ViewDescriptor


@dataclass(frozen=True)
class ScanPose:
    """Pose de escaneo de la trayectoria fija.

    La secuencia no depende del resultado de la medicion: todas las cajas
    ejecutan las mismas tres poses.
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


SCAN_POSES: tuple[ScanPose, ...] = (
    ScanPose("SCAN_YAW_0", 0, 0),
    ScanPose("SCAN_YAW_90", 90, 0),
    ScanPose("SCAN_TILT_35", 0, 35),
)

RETURN_POSE = ScanPose("RETURNED_VERTICAL", 0, 0)


def pose_by_name(name: str) -> ScanPose:
    for pose in SCAN_POSES:
        if pose.name == name:
            return pose
    raise KeyError(name)
