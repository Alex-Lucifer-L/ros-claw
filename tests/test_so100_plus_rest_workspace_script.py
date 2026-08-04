from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys

import mujoco
import numpy as np
import pytest

from rosclaw_mini.arm.kinematics import (
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
    SO100PlusKinematics,
)
from rosclaw_mini.arm.so100_plus_session import (
    SO100_PLUS_MIDDLE_INTERNAL_RADIANS,
)
from rosclaw_mini.workspace_scan import so100_plus as workspace_scan


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT / "scripts" / "simulate_so100_plus_rest_workspace.py"
MODEL_PATH = (
    ROOT
    / "lerobot-joycon_plus"
    / "lerobot"
    / "common"
    / "robot_devices"
    / "controllers"
    / "scene_plus.xml"
)


def _load_compatibility_script_module():
    spec = spec_from_file_location(
        "simulate_so100_plus_rest_workspace",
        SCRIPT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_legacy_script_delegates_to_workspace_scan_package():
    script = _load_compatibility_script_module()

    assert script.main is workspace_scan.main
    assert script.centered_axis_values is workspace_scan.centered_axis_values


def test_centered_axis_values_contains_exact_center():
    script = workspace_scan

    values, center_index = script.centered_axis_values(
        0.303,
        -0.3,
        0.52,
        0.02,
    )

    assert values[center_index] == pytest.approx(0.303)
    assert np.diff(values) == pytest.approx(0.02)
    assert values[0] >= -0.3
    assert values[-1] <= 0.52


def test_explicit_refinement_bounds_must_contain_rest_and_stay_in_candidate():
    script = workspace_scan
    rest = np.asarray((0.30, 0.0, 0.18))
    candidate_lower = np.asarray((-0.34, -0.31, 0.0))
    candidate_upper = np.asarray((0.53, 0.27, 0.55))

    lower, upper, source = script.select_grid_bounds(
        rest,
        candidate_lower,
        candidate_upper,
        (0.15, 0.53, -0.15, 0.15, 0.03, 0.38),
    )

    assert lower == pytest.approx((0.15, -0.15, 0.03))
    assert upper == pytest.approx((0.53, 0.15, 0.38))
    assert source == "explicit_refinement_bounds"

    with pytest.raises(ValueError, match="包含参考 TCP"):
        script.select_grid_bounds(
            rest,
            candidate_lower,
            candidate_upper,
            (0.31, 0.53, -0.15, 0.15, 0.03, 0.38),
        )

    expanded_lower, expanded_upper, _source = script.select_grid_bounds(
        rest,
        candidate_lower,
        candidate_upper,
        (-0.40, 0.60, -0.35, 0.30, 0.0, 0.60),
        allow_outside_candidate_aabb=True,
    )
    assert expanded_lower == pytest.approx((-0.40, -0.35, 0.0))
    assert expanded_upper == pytest.approx((0.60, 0.30, 0.60))


def test_largest_valid_box_contains_center_and_only_true_points():
    script = workspace_scan
    mask = np.zeros((6, 7, 8), dtype=bool)
    mask[1:6, 2:7, 1:7] = True
    center = (3, 4, 4)

    box = script.largest_valid_box_containing_center(mask, center)

    assert box == script.RestCenteredBox(1, 5, 2, 6, 1, 6)
    assert np.all(
        mask[
            box.x_start : box.x_end + 1,
            box.y_start : box.y_end + 1,
            box.z_start : box.z_end + 1,
        ]
    )


def test_largest_valid_box_does_not_cross_internal_hole():
    script = workspace_scan
    mask = np.ones((5, 5, 5), dtype=bool)
    mask[0, 0, 0] = False
    center = (2, 2, 2)

    box = script.largest_valid_box_containing_center(mask, center)
    selected = mask[
        box.x_start : box.x_end + 1,
        box.y_start : box.y_end + 1,
        box.z_start : box.z_end + 1,
    ]

    assert np.all(selected)
    assert box.point_count < mask.size


def test_directed_neighbor_edge_count_and_direction():
    script = workspace_scan
    box = script.RestCenteredBox(0, 1, 0, 2, 0, 3)

    edges = list(script.iter_directed_neighbor_edges(box))

    # Undirected edges: 1*3*4 + 2*2*4 + 2*3*3 = 46.
    assert len(edges) == 92
    assert ((0, 0, 0), (1, 0, 0)) in edges
    assert ((1, 0, 0), (0, 0, 0)) in edges


def test_rest_pose_and_zero_length_path_are_collision_free():
    script = workspace_scan
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    kinematics = SO100PlusKinematics()
    rest_qpos = np.asarray(SO100_PLUS_JOYCON_INITIAL_RADIANS, dtype=float)
    rest_tcp = np.asarray(kinematics.forward_position(rest_qpos))

    assert not script.pose_has_collision(
        model,
        data,
        rest_qpos,
        rest_tcp,
    )
    assert not script.path_has_collision(
        model,
        data,
        kinematics,
        rest_qpos,
        rest_qpos,
        max_step_radians=np.radians(2.0),
    )


@pytest.mark.parametrize("gripper_degrees", (-5.0, 60.0))
def test_middle_reference_pose_is_collision_free_for_runtime_gripper_extremes(
    gripper_degrees,
):
    script = workspace_scan
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    kinematics = SO100PlusKinematics()
    joints = np.asarray(SO100_PLUS_MIDDLE_INTERNAL_RADIANS, dtype=float)
    tcp = np.asarray(kinematics.forward_position(joints))
    gripper_qpos = np.radians(gripper_degrees)

    assert script.REFERENCE_JOINT_RADIANS["middle_internal"] == pytest.approx(
        joints
    )
    assert not script.pose_has_collision(
        model,
        data,
        joints,
        tcp,
        gripper_qpos=gripper_qpos,
    )
