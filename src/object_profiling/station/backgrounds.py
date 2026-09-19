from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from ..measure.background import BackgroundSet, PoseBackground
from .camera import RGBDSensor
from .controller import ScanPoseController
from .environment import ProfilingEnvironment
from .poses import SCAN_POSES, ScanPose


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
