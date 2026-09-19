from __future__ import annotations

import mujoco
import numpy as np

from ..contracts import RejectionReason
from .environment import ProfilingEnvironment


class MotionError(RuntimeError):
    def __init__(self, reason: RejectionReason):
        super().__init__(reason.value)
        self.reason = reason


class ScanPoseController:
    def __init__(self, environment: ProfilingEnvironment):
        self.environment = environment

    def move_to(self, angle_deg: int, *, lifted: bool = True, on_step=None) -> None:
        self.move_to_qpos(
            self.environment.config.motion.target(angle_deg, lifted=lifted),
            on_step=on_step,
        )

    def move_to_qpos(self, target: np.ndarray, *, on_step=None) -> None:
        env = self.environment
        target = np.asarray(target, dtype=np.float64)
        start = env.data.qpos[:6].copy()
        timestep = env.model.opt.timestep
        steps = max(2, int(env.config.motion.trajectory_duration_s / timestep))
        for step in range(steps):
            phase = (step + 1) / steps
            blend = phase * phase * phase * (10.0 + phase * (-15.0 + 6.0 * phase))
            env.data.ctrl[:] = start + blend * (target - start)
            mujoco.mj_step(env.model, env.data)
            if on_step is not None:
                on_step()

        env.data.ctrl[:] = target
        timeout_steps = int(env.config.motion.timeout_s / timestep)
        consecutive = 0
        required = max(5, int(0.05 / timestep))
        for _ in range(timeout_steps):
            mujoco.mj_step(env.model, env.data)
            if on_step is not None:
                on_step()
            error = float(np.max(np.abs(env.data.qpos[:6] - target)))
            velocity = float(np.max(np.abs(env.data.qvel[:6])))
            if error <= env.config.motion.joint_error_rad and velocity <= env.config.motion.joint_velocity_rad_s:
                consecutive += 1
                if consecutive >= required:
                    return
            else:
                consecutive = 0
        raise MotionError(RejectionReason.MOTION_TIMEOUT)
