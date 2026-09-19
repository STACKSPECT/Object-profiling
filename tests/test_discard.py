from __future__ import annotations

from dataclasses import replace

import mujoco
import numpy as np

from object_profiling.config import AppConfig
from object_profiling.contracts import BoxCondition, RoutingHint
from object_profiling.evaluation.checkpoint import NOMINAL_BOX
from object_profiling.station.camera import RGBDSensor
from object_profiling.station.damage import DamageKind, DamageSpec
from object_profiling.station.discard import box_rests_in_error_bin, discard_to_error_zone
from object_profiling.station.environment import ProfilingEnvironment
from object_profiling.station.pipeline import profile
from object_profiling.station.poses import SCAN_POSES


def test_error_bin_is_outside_scan_camera() -> None:
    environment = ProfilingEnvironment.create(NOMINAL_BOX, AppConfig(), attach_box=False)
    environment.set_box_visible(False)
    sensor = RGBDSensor(environment)
    try:
        environment.set_joint_positions(SCAN_POSES[0].target_qpos(environment.config.motion))
        with_bin = sensor.capture("SCAN_YAW_0", yaw_deg=0, tilt_deg=0).depth_m.copy()
        floor = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_GEOM, "error_bin_floor")
        alphas = []
        for name in (
            "error_bin_floor",
            "error_bin_wall_xp",
            "error_bin_wall_xn",
            "error_bin_wall_yp",
            "error_bin_wall_yn",
        ):
            gid = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            alphas.append((gid, float(environment.model.geom_rgba[gid, 3])))
            environment.model.geom_rgba[gid, 3] = 0.0
        without = sensor.capture("SCAN_YAW_0", yaw_deg=0, tilt_deg=0).depth_m
        for gid, alpha in alphas:
            environment.model.geom_rgba[gid, 3] = alpha
        changed = np.abs(with_bin - without) > 5e-4
        assert int(changed.sum()) == 0
        assert floor >= 0
    finally:
        sensor.close()


def test_visible_crushed_corner_is_measured_and_parked() -> None:
    spec = replace(
        NOMINAL_BOX,
        damage=DamageSpec(DamageKind.CRUSHED_CORNER, 0.03, "corner:+x+y+z"),
    )
    environment = ProfilingEnvironment.create(spec, AppConfig(), attach_box=False)
    result = profile(environment)

    assert result.dimensions.valid is True
    assert result.dimensions.condition is BoxCondition.DAMAGED
    assert result.dimensions.routing is RoutingHint.ERROR_ZONE
    assert result.dimensions.damage is not None
    assert result.dimensions.damage.kind.value == "CRUSHED_CORNER"
    assert box_rests_in_error_bin(environment)
    assert not environment.box_attached


def test_visible_dent_is_routed_to_the_error_zone() -> None:
    spec = replace(
        NOMINAL_BOX,
        damage=DamageSpec(DamageKind.DENTED_FACE, 0.025, "face:+x", radius_m=0.05),
    )
    environment = ProfilingEnvironment.create(spec, AppConfig(), attach_box=False)
    result = profile(environment)

    assert result.dimensions.valid is True
    assert result.dimensions.condition is BoxCondition.DAMAGED
    assert result.dimensions.routing is RoutingHint.ERROR_ZONE
    assert result.dimensions.damage is not None
    assert result.dimensions.dimensions_m is not None
    truth = spec.dimensions_m.as_array()
    estimated = result.dimensions.dimensions_m.as_array()
    assert np.all(np.abs(estimated - truth) < 0.005)
    assert box_rests_in_error_bin(environment)
    assert not environment.box_attached
    environment = ProfilingEnvironment.create(NOMINAL_BOX, AppConfig(), attach_box=True)
    environment.set_joint_positions(np.asarray(environment.config.motion.lift_qpos))
    environment._place_box()
    environment.attach_box()
    discard_to_error_zone(environment)
    assert box_rests_in_error_bin(environment)
    assert not environment.box_attached


def test_bottom_panel_buckle_is_measured_and_parked() -> None:
    spec = replace(
        NOMINAL_BOX,
        damage=DamageSpec(DamageKind.BUCKLED_PANEL, 0.020, "face:+z"),
    )
    environment = ProfilingEnvironment.create(spec, AppConfig(), attach_box=False)
    result = profile(environment)

    assert result.dimensions.valid is True
    assert result.dimensions.condition is BoxCondition.DAMAGED
    assert result.dimensions.routing is RoutingHint.ERROR_ZONE
    assert result.dimensions.dimensions_m is not None
    truth = spec.dimensions_m.as_array()
    estimated = result.dimensions.dimensions_m.as_array()
    assert np.all(np.abs(estimated - truth) < 0.005)
    assert box_rests_in_error_bin(environment)
    assert not environment.box_attached
