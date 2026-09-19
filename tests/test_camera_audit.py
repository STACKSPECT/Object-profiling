from __future__ import annotations

import mujoco
import numpy as np
import pytest

from object_profiling.evaluation.audits.camera import project_box_corners
from object_profiling.evaluation.checkpoint import MAXIMUM_BOX, MINIMUM_BOX, NOMINAL_BOX
from object_profiling.station.controller import ScanPoseController
from object_profiling.station.environment import ProfilingEnvironment
from object_profiling.station.poses import INSPECTION_POSES, SCAN_POSES


def test_fixed_camera_configuration() -> None:
    environment = ProfilingEnvironment.create(NOMINAL_BOX, attach_box=False)
    camera_id = mujoco.mj_name2id(environment.model, mujoco.mjtObj.mjOBJ_CAMERA, "scan_rgbd_cam")
    position = np.asarray(environment.data.cam_xpos[camera_id], dtype=float)
    target = np.asarray(environment.config.sensor.scan_center_world_m, dtype=float)

    assert position == pytest.approx((0.5566, 1.22835, 0.38025))
    assert float(position[2]) < float(target[2])
    assert environment.model.cam_fovy[camera_id] == pytest.approx(50.0)
    assert (environment.config.sensor.width, environment.config.sensor.height) == (640, 480)


@pytest.mark.parametrize("box_spec", [MINIMUM_BOX, NOMINAL_BOX, MAXIMUM_BOX])
def test_all_box_corners_remain_inside_frame_for_fixed_scan(box_spec) -> None:
    environment = ProfilingEnvironment.create(box_spec, attach_box=False)
    environment.attach_box()
    controller = ScanPoseController(environment)

    for pose in (*SCAN_POSES, *INSPECTION_POSES):
        controller.move_to_qpos(pose.target_qpos(environment.config.motion))
        pixels = project_box_corners(environment)

        assert np.all(pixels[:, 2] > 0.0)
        assert float(pixels[:, 0].min()) >= 40.0
        assert float(pixels[:, 0].max()) <= environment.config.sensor.width - 1 - 40.0
        assert float(pixels[:, 1].min()) >= 40.0
        assert float(pixels[:, 1].max()) <= environment.config.sensor.height - 1 - 40.0
