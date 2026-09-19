from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from ..contracts import CameraObservation
from ..measure.background import BackgroundSet
from .backgrounds import capture_pose_backgrounds
from .camera import RGBDSensor
from .controller import ScanPoseController
from .environment import ProfilingEnvironment
from .poses import RETURN_POSE, SCAN_POSES, ScanPose


@dataclass(frozen=True)
class ScanCycle:
    backgrounds: BackgroundSet
    observations: tuple[CameraObservation, ...]


def calibrate_backgrounds(
    environment: ProfilingEnvironment,
    sensor: RGBDSensor,
    *,
    poses: tuple[ScanPose, ...] = SCAN_POSES,
) -> BackgroundSet:
    """Calibra los fondos de la estacion vacia.

    Ya no depende de la caja del episodio, porque el apoyo esta a una altura fija,
    asi que el resultado se puede reutilizar en todos los ciclos de una sesion.
    """

    environment.set_box_visible(False)
    backgrounds = capture_pose_backgrounds(environment, sensor, poses=poses)
    environment.reset(attach_box=False)
    return backgrounds


def run_fixed_scan(
    environment: ProfilingEnvironment,
    sensor: RGBDSensor,
    *,
    poses: tuple[ScanPose, ...] = SCAN_POSES,
    backgrounds: BackgroundSet | None = None,
    on_state: Callable[[str], None] | None = None,
    on_step: Callable[[], None] | None = None,
    on_capture: Callable[[ScanPose, CameraObservation, BackgroundSet], None] | None = None,
) -> ScanCycle:
    """Ejecuta el ciclo fijo de agarre, escaneo y retorno.

    La secuencia no depende de lo que se observe: todas las cajas recorren las
    mismas poses. Si se pasan fondos ya calibrados, el ciclo no retira la caja en
    ningun momento; si no, los calibra antes de agarrar.
    """

    report = on_state or (lambda _name: None)
    controller = ScanPoseController(environment)

    if backgrounds is None:
        report("CALIBRATE_BACKGROUND")
        backgrounds = calibrate_backgrounds(environment, sensor, poses=poses)

    report("PRESENT_BOX")
    environment.reset(attach_box=False)

    report("ATTACH_SUCTION")
    environment.attach_box()

    observations: list[CameraObservation] = []
    for pose in poses:
        report(pose.name)
        controller.move_to_qpos(pose.target_qpos(environment.config.motion), on_step=on_step)
        observation = sensor.capture(pose.name, yaw_deg=pose.yaw_deg, tilt_deg=pose.tilt_deg)
        observations.append(observation)
        if on_capture is not None:
            on_capture(pose, observation, backgrounds)

    report(RETURN_POSE.name)
    controller.move_to_qpos(RETURN_POSE.target_qpos(environment.config.motion), on_step=on_step)
    return ScanCycle(backgrounds, tuple(observations))
