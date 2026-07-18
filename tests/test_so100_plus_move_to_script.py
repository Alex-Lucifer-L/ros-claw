import pytest

from scripts.check_so100_plus_adapter_move_to import (
    _validate_single_cartesian_plan,
    build_parser,
)


class FakeKinematics:
    def forward_position(self, waypoint):
        return tuple(waypoint[:3])


def test_real_move_script_defaults_to_saved_hardware_profile():
    args = build_parser().parse_args(
        [
            "--port",
            "/dev/lerobot_right",
            "--calibration-dir",
            "calibration",
            "--follower-name",
            "right",
        ]
    )

    assert args.runtime_acceleration == 35
    assert args.stream_frequency_hz == 30.0
    assert args.stream_max_joint_speed_degrees_per_second == 20.0


def test_single_plan_path_accepts_small_upward_steps():
    max_step, max_lateral = _validate_single_cartesian_plan(
        FakeKinematics(),
        (0.0, 0.0, 0.0),
        (
            (0.001, 0.0, 0.003, 0.0, 0.0, 0.0),
            (0.002, 0.0, 0.006, 0.0, 0.0, 0.0),
        ),
    )

    assert max_step == pytest.approx(0.0031622776601683794)
    assert max_lateral == pytest.approx(0.002)


def test_single_plan_path_rejects_downward_internal_step():
    with pytest.raises(RuntimeError, match="不向上"):
        _validate_single_cartesian_plan(
            FakeKinematics(),
            (0.0, 0.0, 0.0),
            (
                (0.0, 0.0, 0.003, 0.0, 0.0, 0.0),
                (0.0, 0.0, 0.002, 0.0, 0.0, 0.0),
            ),
        )


def test_single_plan_path_rejects_excessive_lateral_deviation():
    with pytest.raises(RuntimeError, match="横向偏移"):
        _validate_single_cartesian_plan(
            FakeKinematics(),
            (0.0, 0.0, 0.0),
            (
                (0.003, 0.0, 0.001, 0.0, 0.0, 0.0),
                (0.006, 0.0, 0.002, 0.0, 0.0, 0.0),
            ),
        )
