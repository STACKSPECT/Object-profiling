from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

import mujoco
import numpy as np

from ..config import AppConfig
from ..contracts import Dimensions3D
from ..station.controller import MotionError, ScanPoseController
from ..station.environment import BoxSpec, ProfilingEnvironment
from ..station.poses import RETURN_POSE, SCAN_POSES


NOMINAL_BOX = BoxSpec(
    object_id="checkpoint-box",
    dimensions_m=Dimensions3D(length=0.30, width=0.20, height=0.15),
    mass_kg=2.0,
    rgba=(0.72, 0.43, 0.19, 1.0),
)
MINIMUM_BOX = BoxSpec(
    object_id="checkpoint-box-minimum",
    dimensions_m=Dimensions3D(length=0.15, width=0.12, height=0.08),
    mass_kg=0.5,
    rgba=(0.72, 0.43, 0.19, 1.0),
)
MAXIMUM_BOX = BoxSpec(
    object_id="checkpoint-box-maximum",
    dimensions_m=Dimensions3D(length=0.40, width=0.30, height=0.25),
    mass_kg=5.0,
    rgba=(0.72, 0.43, 0.19, 1.0),
)


def _positive_speed(value: str) -> float:
    speed = float(value)
    if not math.isfinite(speed) or speed <= 0.0:
        raise argparse.ArgumentTypeError("--speed debe ser un numero finito mayor que 0")
    return speed


@dataclass(frozen=True)
class CheckpointState:
    name: str
    simulation_time_s: float
    target_yaw_deg: int
    target_tilt_deg: int
    qpos_rad: list[float]
    max_joint_error_rad: float
    box_position_world_m: list[float]
    tool_orientation_change_deg: float
    translation_drift_m: float
    rotation_drift_deg: float
    box_attached: bool
    unexpected_box_contact_samples: int


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.rad2deg(np.arccos(cosine)))


def _capture_state(
    environment: ProfilingEnvironment,
    name: str,
    target_yaw_deg: int,
    target_tilt_deg: int,
    target_qpos: np.ndarray,
    reference_gripper_to_box: np.ndarray,
    reference_tool_to_world: np.ndarray,
    unexpected_box_contact_samples: int,
) -> CheckpointState:
    current = environment.gripper_to_box()
    delta = np.linalg.inv(reference_gripper_to_box) @ current
    box_to_world = environment.body_to_world("profiling_box")
    tool_to_world = environment.tool_to_world()
    tool_rotation_delta = reference_tool_to_world[:3, :3].T @ tool_to_world[:3, :3]
    return CheckpointState(
        name=name,
        simulation_time_s=float(environment.data.time),
        target_yaw_deg=target_yaw_deg,
        target_tilt_deg=target_tilt_deg,
        qpos_rad=[float(value) for value in environment.data.qpos[:6]],
        max_joint_error_rad=float(np.max(np.abs(environment.data.qpos[:6] - target_qpos))),
        box_position_world_m=[float(value) for value in box_to_world[:3, 3]],
        tool_orientation_change_deg=_rotation_angle_deg(tool_rotation_delta),
        translation_drift_m=float(np.linalg.norm(delta[:3, 3])),
        rotation_drift_deg=_rotation_angle_deg(delta[:3, :3]),
        box_attached=environment.box_attached,
        unexpected_box_contact_samples=unexpected_box_contact_samples,
    )


def _box_has_contact(environment: ProfilingEnvironment) -> bool:
    box_geom_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    return any(
        environment.data.contact[index].geom1 == box_geom_id
        or environment.data.contact[index].geom2 == box_geom_id
        for index in range(environment.data.ncon)
    )


def _execute_checkpoint(
    environment: ProfilingEnvironment,
    *,
    seed: int,
    on_step: Callable[[], None] | None = None,
    on_state: Callable[[str], None] | None = None,
) -> dict:
    controller = ScanPoseController(environment)
    report_state = on_state or (lambda _name: None)

    initial_box_z = float(environment.body_to_world("profiling_box")[2, 3])
    report_state("ATTACH_SUCTION")
    environment.attach_box()
    reference = environment.gripper_to_box().copy()
    reference_tool = environment.tool_to_world().copy()

    states: list[CheckpointState] = []
    failure_reason: str | None = None

    def execute_pose(
        command_name: str,
        state_name: str,
        target_qpos: np.ndarray,
        *,
        yaw_deg: int,
        tilt_deg: int,
        allow_support_contact: bool = False,
    ) -> None:
        report_state(command_name)
        contact_samples = 0

        def audit_step() -> None:
            nonlocal contact_samples
            if not allow_support_contact and _box_has_contact(environment):
                contact_samples += 1
            if on_step is not None:
                on_step()

        controller.move_to_qpos(target_qpos, on_step=audit_step)
        states.append(
            _capture_state(
                environment,
                state_name,
                yaw_deg,
                tilt_deg,
                target_qpos,
                reference,
                reference_tool,
                contact_samples,
            )
        )

    motion = environment.config.motion
    commands = (
        ("LIFT", SCAN_POSES[0], True),
        ("ROTATE_YAW_90", SCAN_POSES[1], False),
        ("RETURN_VERTICAL", RETURN_POSE, False),
    )

    try:
        for command_name, pose, allow_support_contact in commands:
            execute_pose(
                command_name,
                pose.name,
                pose.target_qpos(motion),
                yaw_deg=pose.yaw_deg,
                tilt_deg=pose.tilt_deg,
                allow_support_contact=allow_support_contact,
            )
    except MotionError as error:
        failure_reason = error.reason.value

    final_box_z = float(environment.body_to_world("profiling_box")[2, 3])
    max_translation_drift = max((state.translation_drift_m for state in states), default=float("inf"))
    max_rotation_drift = max((state.rotation_drift_deg for state in states), default=float("inf"))
    max_joint_error = max((state.max_joint_error_rad for state in states), default=float("inf"))
    lifted_distance = final_box_z - initial_box_z
    expected_sequence = [pose.name for pose in SCAN_POSES] + [RETURN_POSE.name]
    completed_sequence = [state.name for state in states] == expected_sequence
    unexpected_contact_samples = sum(state.unexpected_box_contact_samples for state in states)
    success = bool(
        failure_reason is None
        and completed_sequence
        and environment.box_attached
        and lifted_distance >= 0.10
        and max_translation_drift <= 0.001
        and max_rotation_drift <= 0.5
        and max_joint_error <= environment.config.motion.joint_error_rad
        and unexpected_contact_samples == 0
    )

    return {
        "schema_version": 2,
        "seed": seed,
        "checkpoint": "grasp_lift_yaw_tilt_return",
        "success": success,
        "failure_reason": failure_reason if failure_reason else (None if success else "AUDIT_THRESHOLD_FAILED"),
        "assumptions": {
            "box_dimensions_m": asdict(environment.box_spec.dimensions_m),
            "box_mass_kg": environment.box_spec.mass_kg,
            "grasp_model": "declared_rigid_equality_weld",
            "fixed_motion": True,
            "tilt_angle_deg": 0,
            "pickup_perception": False,
            "measurement": False,
        },
        "active_cups": list(environment.active_cup_names()),
        "lifted_distance_m": lifted_distance,
        "max_translation_drift_m": max_translation_drift,
        "max_rotation_drift_deg": max_rotation_drift,
        "max_joint_error_rad": max_joint_error,
        "unexpected_box_contact_samples": unexpected_contact_samples,
        "states": [asdict(state) for state in states],
    }


def run_checkpoint(
    *,
    seed: int = 42,
    box_spec: BoxSpec = NOMINAL_BOX,
    on_step: Callable[[], None] | None = None,
    on_state: Callable[[str], None] | None = None,
) -> dict:
    """Ejecuta el checkpoint fijo de agarre, yaw, inclinacion y retorno."""

    environment = ProfilingEnvironment.create(box_spec, AppConfig(), attach_box=False)
    return _execute_checkpoint(
        environment,
        seed=seed,
        on_step=on_step,
        on_state=on_state,
    )


def _write_report(report: dict, output: Path | None) -> None:
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text + "\n", encoding="utf-8")


def _run_visual(seed: int, speed: float) -> dict:
    import mujoco.viewer

    environment = ProfilingEnvironment.create(NOMINAL_BOX, AppConfig(), attach_box=False)

    with mujoco.viewer.launch_passive(environment.model, environment.data) as viewer:
        def animate_step() -> None:
            viewer.sync()
            time.sleep(environment.model.opt.timestep / speed)

        def announce(name: str) -> None:
            print(f"[checkpoint] {name}")
            viewer.sync()
            time.sleep(0.6 / speed)

        report = _execute_checkpoint(
            environment,
            seed=seed,
            on_step=animate_step,
            on_state=announce,
        )
        announce("DONE")
        return report


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Checkpoint: agarrar, elevar, girar e inclinar una caja nominal con el UR10e."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--visual", action="store_true", help="Muestra la ejecucion en el visor de MuJoCo.")
    mode.add_argument("--headless", action="store_true", help="Ejecuta sin abrir una ventana (modo por defecto).")
    parser.add_argument("--seed", type=int, default=42, help="Seed registrada; la escena de este checkpoint es fija.")
    parser.add_argument(
        "--speed",
        type=_positive_speed,
        default=1.0,
        metavar="MULTIPLICADOR",
        help="Velocidad de reproduccion visual: 1.0 real, 2.0 doble, 0.5 mitad.",
    )
    parser.add_argument("--output", type=Path, help="Ruta opcional para guardar el informe JSON.")
    args = parser.parse_args()

    if args.visual:
        report = _run_visual(args.seed, args.speed)
    else:
        report = run_checkpoint(seed=args.seed, on_state=lambda name: print(f"[checkpoint] {name}"))
    _write_report(report, args.output)
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
