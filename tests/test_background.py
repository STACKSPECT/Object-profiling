from __future__ import annotations

import mujoco
import numpy as np
import pytest

from object_profiling.background import (
    BackgroundSet,
    MissingBackgroundError,
    PoseBackground,
    capture_pose_backgrounds,
)
from object_profiling.checkpoint import NOMINAL_BOX
from object_profiling.contracts import RejectionReason
from object_profiling.environment import BOX_PARKING_POSITION_M, ProfilingEnvironment
from object_profiling.poses import SCAN_POSES
from object_profiling.sensors import RGBDSensor


def _empty_station() -> ProfilingEnvironment:
    environment = ProfilingEnvironment.create(NOMINAL_BOX, attach_box=False)
    environment.set_box_visible(False)
    return environment


def _capture() -> BackgroundSet:
    environment = _empty_station()
    sensor = RGBDSensor(environment)
    try:
        return capture_pose_backgrounds(environment, sensor)
    finally:
        sensor.close()


def test_hidden_box_leaves_the_station_without_contacts() -> None:
    environment = _empty_station()

    assert not environment.box_visible
    assert not environment.box_attached
    position = environment.body_to_world("profiling_box")[:3, 3]
    assert position == pytest.approx(BOX_PARKING_POSITION_M)

    for _ in range(300):
        mujoco.mj_step(environment.model, environment.data)

    assert environment.data.ncon == 0
    assert float(np.max(np.abs(environment.data.qvel[:6]))) < 1e-3


def test_reset_restores_the_box_after_hiding() -> None:
    environment = _empty_station()
    environment.reset(attach_box=False)

    assert environment.box_visible
    geom_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    assert environment.model.geom_contype[geom_id] == 1
    assert environment.model.geom_conaffinity[geom_id] == 1


def test_backgrounds_require_an_empty_station() -> None:
    environment = ProfilingEnvironment.create(NOMINAL_BOX, attach_box=False)
    sensor = RGBDSensor(environment)
    try:
        with pytest.raises(ValueError):
            capture_pose_backgrounds(environment, sensor)
    finally:
        sensor.close()


def test_backgrounds_cover_every_scan_pose() -> None:
    backgrounds = _capture()

    assert backgrounds.pose_names == tuple(pose.name for pose in SCAN_POSES)
    assert backgrounds.covers(pose.name for pose in SCAN_POSES)


def test_background_depth_shape_dtype_and_validity() -> None:
    backgrounds = _capture()

    for pose in SCAN_POSES:
        depth = backgrounds.depth_for(pose.name)
        assert depth.shape == (480, 640)
        assert depth.dtype == np.float64
        assert np.isfinite(depth).all()
        assert float(depth.min()) > 0.0


def test_each_pose_needs_its_own_background() -> None:
    backgrounds = _capture()
    yaw_0 = backgrounds.depth_for("SCAN_YAW_0")

    for pose_name in ("SCAN_YAW_90", "SCAN_TILT_35"):
        differing = int(np.count_nonzero(np.abs(backgrounds.depth_for(pose_name) - yaw_0) > 1e-4))
        assert differing > 5_000, f"{pose_name} reutilizaria el fondo de SCAN_YAW_0"


def test_missing_background_reports_an_explicit_rejection() -> None:
    backgrounds = _capture()

    assert not backgrounds.covers(("SCAN_YAW_180",))
    with pytest.raises(MissingBackgroundError) as error:
        backgrounds.depth_for("SCAN_YAW_180")
    assert error.value.reason is RejectionReason.MISSING_BACKGROUND
    assert error.value.pose_name == "SCAN_YAW_180"


def test_background_capture_is_deterministic() -> None:
    first, second = _capture(), _capture()

    for pose in SCAN_POSES:
        np.testing.assert_array_equal(first.depth_for(pose.name), second.depth_for(pose.name))


def test_recorded_joint_positions_reproduce_the_background_pose() -> None:
    """Colocar el brazo en una configuracion medida reproduce su fondo.

    El residuo no es nulo: el terminal cuelga del weld y se asienta unas
    centesimas de milimetro por debajo de su posicion cinematica. Eso conmuta
    los pixeles de silueta de las copas entre la copa y el suelo lejano, asi que
    el limite se comprueba sobre el grueso de la imagen, no sobre el maximo.
    """

    reference = _capture()
    recorded = {pose.name: reference.joint_positions_for(pose.name) for pose in SCAN_POSES}

    environment = _empty_station()
    sensor = RGBDSensor(environment)
    try:
        aligned = capture_pose_backgrounds(environment, sensor, joint_positions_rad=recorded)
    finally:
        sensor.close()

    for pose in SCAN_POSES:
        assert aligned.joint_positions_for(pose.name) == pytest.approx(recorded[pose.name])
        deviation = np.abs(aligned.depth_for(pose.name) - reference.depth_for(pose.name))
        assert float(np.percentile(deviation, 99.0)) < 1e-4
        assert int(np.count_nonzero(deviation > 1e-6)) < deviation.size // 100


def test_background_set_rejects_lookup_on_empty_calibration() -> None:
    empty = BackgroundSet(())

    assert empty.pose_names == ()
    with pytest.raises(MissingBackgroundError):
        empty.depth_for("SCAN_YAW_0")


def test_pose_background_keeps_pose_name_and_depth_together() -> None:
    depth = np.full((4, 4), 1.5)
    background = PoseBackground("SCAN_YAW_0", depth, np.zeros(6))

    assert BackgroundSet((background,)).depth_for("SCAN_YAW_0") is depth
