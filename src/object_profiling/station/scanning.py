from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from ..contracts import CameraObservation
from ..measure.background import BackgroundSet
from .backgrounds import capture_pose_backgrounds
from .camera import RGBDSensor
from .controller import ScanPoseController
from .environment import ProfilingEnvironment
from .poses import INSPECTION_POSES, SCAN_POSES, ScanPose


@dataclass(frozen=True)
class ScanCycle:
    backgrounds: BackgroundSet
    observations: tuple[CameraObservation, ...]
    # RGB-D de inspeccion (yaw 180). Se fusiona para dano, no para L/W/H.
    inspection_observations: tuple[CameraObservation, ...] = ()


def _unique_poses(*groups: tuple[ScanPose, ...]) -> tuple[ScanPose, ...]:
    seen: set[str] = set()
    ordered: list[ScanPose] = []
    for group in groups:
        for pose in group:
            if pose.name in seen:
                continue
            seen.add(pose.name)
            ordered.append(pose)
    return tuple(ordered)


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
    finish_poses: tuple[ScanPose, ...] | None = None,
    backgrounds: BackgroundSet | None = None,
    on_state: Callable[[str], None] | None = None,
    on_step: Callable[[], None] | None = None,
    on_capture: Callable[[ScanPose, CameraObservation, BackgroundSet], None] | None = None,
) -> ScanCycle:
    """Agarre, capturas de medida y giro de inspeccion en el mismo sentido.

    `poses` alimenta measure(). `finish_poses` (por defecto yaw 180) se fusiona
    solo para inspeccion de las cinco caras visibles; no entra en L/W/H.
    """

    report = on_state or (lambda _name: None)
    controller = ScanPoseController(environment)
    inspection_poses = INSPECTION_POSES if finish_poses is None else finish_poses
    background_poses = _unique_poses(poses, inspection_poses)

    if backgrounds is None:
        report("CALIBRATE_BACKGROUND")
        backgrounds = calibrate_backgrounds(environment, sensor, poses=background_poses)

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

    inspection_observations: list[CameraObservation] = []
    for pose in inspection_poses:
        report(pose.name)
        controller.move_to_qpos(pose.target_qpos(environment.config.motion), on_step=on_step)
        observation = sensor.capture(pose.name, yaw_deg=pose.yaw_deg, tilt_deg=pose.tilt_deg)
        inspection_observations.append(observation)

    return ScanCycle(backgrounds, tuple(observations), tuple(inspection_observations))
