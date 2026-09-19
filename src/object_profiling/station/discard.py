"""IK y secuencia de descarte hacia el contenedor de rechazo."""

from __future__ import annotations

from collections.abc import Callable

import mujoco
import numpy as np

from .controller import ScanPoseController
from .environment import ProfilingEnvironment


def solve_site_position(
    environment: ProfilingEnvironment,
    site_name: str,
    target_position_m: np.ndarray,
    *,
    iterations: int = 80,
    damping: float = 1e-3,
) -> np.ndarray:
    """DLS sobre los seis ejes del UR10e, sin cambiar la orientacion buscada mas
    que lo que el jacobiano de posicion arrastre. Parte de la pose elevada."""

    env = environment
    site_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    env.data.qpos[:6] = np.asarray(env.config.motion.lift_qpos, dtype=np.float64)
    env.data.qvel[:] = 0.0
    env.data.ctrl[:] = env.data.qpos[:6]
    jacp = np.zeros((3, env.model.nv))
    target = np.asarray(target_position_m, dtype=np.float64)
    for _ in range(iterations):
        mujoco.mj_forward(env.model, env.data)
        error = target - env.data.site_xpos[site_id]
        if float(np.linalg.norm(error)) < 1e-3:
            break
        mujoco.mj_jacSite(env.model, env.data, jacp, None, site_id)
        jacobian = jacp[:, :6]
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * np.eye(3), error)
        env.data.qpos[:6] += delta
        for index in range(6):
            lower, upper = env.model.jnt_range[index]
            two_pi = 2.0 * np.pi
            value = float(env.data.qpos[index])
            while value > upper:
                value -= two_pi
            while value < lower:
                value += two_pi
            env.data.qpos[index] = np.clip(value, lower, upper)
        env.data.ctrl[:] = env.data.qpos[:6]
    mujoco.mj_forward(env.model, env.data)
    start = np.asarray(env.config.motion.lift_qpos, dtype=np.float64)
    two_pi = 2.0 * np.pi
    for index in range(6):
        lower, upper = env.model.jnt_range[index]
        candidates = [float(env.data.qpos[index]) + k * two_pi for k in range(-3, 4)]
        valid = [value for value in candidates if lower - 1e-6 <= value <= upper + 1e-6]
        env.data.qpos[index] = min(valid, key=lambda value: abs(value - start[index]))
    env.data.ctrl[:] = env.data.qpos[:6]
    mujoco.mj_forward(env.model, env.data)
    return env.data.qpos[:6].copy()


def error_zone_qpos(environment: ProfilingEnvironment) -> np.ndarray:
    site_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_SITE, "error_zone_drop")
    target = environment.data.site_xpos[site_id].copy()
    saved_qpos = environment.data.qpos.copy()
    saved_ctrl = environment.data.ctrl.copy()
    saved_qvel = environment.data.qvel.copy()
    solved = solve_site_position(environment, "attachment_site", target)
    environment.data.qpos[:] = saved_qpos
    environment.data.ctrl[:] = saved_ctrl
    environment.data.qvel[:] = saved_qvel
    mujoco.mj_forward(environment.model, environment.data)
    return solved


def discard_to_error_zone(
    environment: ProfilingEnvironment,
    *,
    on_state: Callable[[str], None] | None = None,
    on_step: Callable[[], None] | None = None,
) -> None:
    report = on_state or (lambda _name: None)
    controller = ScanPoseController(environment)
    report("MOVE_TO_ERROR_ZONE")
    try:
        controller.move_to_qpos(
            error_zone_qpos(environment),
            on_step=on_step,
            duration_s=3.5,
            timeout_s=6.0,
        )
    finally:
        # Aunque el movimiento expire sobre el contenedor, hay que soltar:
        # dejar el weld activo deja la caja colgada y la politica no se cumple.
        report("RELEASE")
        environment.detach_box()
        settle_steps = int(0.8 / environment.model.opt.timestep)
        for _ in range(settle_steps):
            mujoco.mj_step(environment.model, environment.data)
            if on_step is not None:
                on_step()
    report("RETURN_HOME")
    controller.move_to_qpos(np.asarray(environment.config.motion.home_qpos), on_step=on_step)


def box_rests_in_error_bin(environment: ProfilingEnvironment, margin_m: float = 0.04) -> bool:
    box = environment.body_to_world("profiling_box")[:3, 3]
    site_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_SITE, "error_zone_drop")
    drop = environment.data.site_xpos[site_id]
    floor_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_GEOM, "error_bin_floor")
    half = environment.model.geom_size[floor_id]
    return bool(
        abs(box[0] - drop[0]) <= half[0] - margin_m
        and abs(box[1] - drop[1]) <= half[1] - margin_m
        and box[2] < drop[2]
    )
