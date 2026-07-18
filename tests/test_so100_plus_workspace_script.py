from importlib.util import module_from_spec, spec_from_file_location
import math
from pathlib import Path
import sys

import mujoco
import numpy as np
import pytest

from rosclaw_mini.arm.kinematics import (
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
    SO100PlusKinematics,
)


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "simulate_so100_plus_workspace.py"
)
MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "lerobot-joycon_plus"
    / "lerobot"
    / "common"
    / "robot_devices"
    / "controllers"
    / "scene_plus.xml"
)


def _load_script_module():
    spec = spec_from_file_location(
        "simulate_so100_plus_workspace",
        SCRIPT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_simulation_ranges_keep_measured_driver_base_range():
    script = _load_script_module()
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    lower, upper = script.build_simulation_joint_ranges(model)

    assert math.degrees(lower[0]) == pytest.approx(-31.201172)
    assert math.degrees(upper[0]) == pytest.approx(19.599609)
    assert lower[1:] == pytest.approx(model.jnt_range[1:6, 0])
    assert upper[1:] == pytest.approx(model.jnt_range[1:6, 1])


def test_batch_tcp_positions_match_single_position_fk():
    script = _load_script_module()
    kinematics = SO100PlusKinematics()
    joints = np.asarray(
        [
            SO100_PLUS_JOYCON_INITIAL_RADIANS,
            (0.1, -2.7, 2.4, 0.2, -0.1, 1.0),
            (-0.2, -1.8, 1.2, -0.3, 0.4, -0.8),
        ],
        dtype=float,
    )

    batch_positions = script.batch_tcp_positions(kinematics, joints)
    single_positions = np.asarray(
        [
            kinematics.forward_position(joint_values)
            for joint_values in joints
        ]
    )

    assert batch_positions == pytest.approx(single_positions, abs=1e-12)


def test_mujoco_rest_pose_is_collision_free():
    script = _load_script_module()
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    kinematics = SO100PlusKinematics()
    joints = np.asarray([SO100_PLUS_JOYCON_INITIAL_RADIANS], dtype=float)
    tcp_positions = script.batch_tcp_positions(kinematics, joints)

    result = script.evaluate_collision_free(
        model,
        data,
        joints,
        tcp_positions,
    )

    assert result.collision_free_mask.tolist() == [True]
    assert result.mujoco_collision_samples == 0
    assert result.below_floor_samples == 0


def test_voxelization_deduplicates_points_in_same_voxel():
    script = _load_script_module()
    points = np.asarray(
        [
            (0.001, 0.001, 0.001),
            (0.009, 0.009, 0.009),
            (0.011, 0.001, 0.001),
        ]
    )

    origin, occupied = script.voxelize_points(points, 0.01)

    assert origin == pytest.approx((0.0, 0.0, 0.0))
    assert {tuple(values) for values in occupied.tolist()} == {
        (0, 0, 0),
        (1, 0, 0),
    }
