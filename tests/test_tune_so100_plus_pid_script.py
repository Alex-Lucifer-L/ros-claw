from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT / "scripts"
SCRIPT_PATH = SCRIPTS_DIR / "tune_so100_plus_pid.py"


def _load_script_module():
    scripts_path = str(SCRIPTS_DIR)
    if scripts_path not in sys.path:
        sys.path.insert(0, scripts_path)
    spec = spec_from_file_location("tune_so100_plus_pid", SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _trial(
    script,
    *,
    name,
    i,
    d,
    error_m,
    tcp_span_m=0.0,
    joint_span_degrees=0.0,
):
    return script.PIDTrialResult(
        candidate=script.PIDCandidate(name, i=i, d=d),
        tcp_error_m=error_m,
        max_joint_span_degrees=joint_span_degrees,
        max_tcp_span_m=tcp_span_m,
        max_load=100.0,
        max_temperature_celsius=30.0,
        nominal_target_driver_degrees=(1.0,) * 6,
        actual_driver_degrees=(1.0,) * 6,
    )


def test_pid_search_is_bounded_to_exactly_five_documented_candidates():
    script = _load_script_module()

    derivative = script.derivative_candidates()
    integral = script.integral_candidates(best_d=16)
    candidates = derivative + integral

    assert len(candidates) == script.MAX_PID_TRIALS == 5
    assert [(candidate.i, candidate.d) for candidate in candidates] == [
        (0, 0),
        (0, 16),
        (0, 32),
        (1, 16),
        (2, 16),
    ]


def test_candidate_changes_only_i_and_d_while_preserving_saved_p_values():
    script = _load_script_module()
    candidate = script.PIDCandidate("test", i=2, d=16)

    gains = script.gains_for_candidate(candidate)

    assert {
        name: (value.p, value.i, value.d)
        for name, value in gains.items()
    } == {
        "shoulder_rotation_joint": (16, 2, 16),
        "ellbow_joint": (64, 2, 16),
        "wrist_pitch_joint": (24, 2, 16),
    }


def test_final_ab_mode_has_exactly_two_i2_candidates():
    script = _load_script_module()

    candidates = script.finalist_candidates()

    assert [(candidate.i, candidate.d) for candidate in candidates] == [
        (2, 16),
        (2, 32),
    ]


def test_fixed_ab_target_is_solved_once_from_ideal_joycon_initial_pose():
    script = _load_script_module()
    checkpoint = script.ValidationCheckpoint(
        "near_internal",
        (0.33, 0.0, 0.21),
    )

    class FakeKinematics:
        position_tolerance_m = 0.0001

        def __init__(self):
            self.solve_calls = []

        def solve_position(
            self,
            current_joint_radians,
            target_position_m,
            *,
            joint_limits,
        ):
            self.solve_calls.append(
                (
                    tuple(current_joint_radians),
                    tuple(target_position_m),
                    joint_limits,
                )
            )
            return (0.1,) * 6

        def forward_position(self, _joint_radians):
            return checkpoint.position_m

    kinematics = FakeKinematics()
    joint_limits = object()

    target = script.build_fixed_validation_target(
        kinematics=kinematics,
        checkpoint=checkpoint,
        joint_limits=joint_limits,
    )

    assert target == (0.1,) * 6
    assert kinematics.solve_calls == [
        (
            tuple(script.SO100_PLUS_JOYCON_INITIAL_RADIANS),
            checkpoint.position_m,
            joint_limits,
        )
    ]


def test_fixed_ab_workspace_contains_the_whole_joint_path_with_margin():
    script = _load_script_module()

    class FakeKinematics:
        @staticmethod
        def forward_position(joints):
            return (
                float(joints[0]),
                float(joints[1]),
                0.2 + float(joints[2]) * 0.01,
            )

    target = (0.1, -2.9, 2.8, 0.1, 0.1, 1.5)
    workspace = script.build_fixed_ab_workspace(
        kinematics=FakeKinematics(),
        target_joint_radians=target,
        margin_m=0.015,
    )

    start_position = FakeKinematics.forward_position(
        script.SO100_PLUS_JOYCON_INITIAL_RADIANS
    )
    target_position = FakeKinematics.forward_position(target)
    workspace.validate_position(*start_position)
    workspace.validate_position(*target_position)
    assert workspace.z.minimum >= 0.0


def test_best_trial_prefers_tcp_accuracy_then_low_jitter():
    script = _load_script_module()
    noisier = _trial(
        script,
        name="noisier",
        i=0,
        d=16,
        error_m=0.006,
        tcp_span_m=0.002,
    )
    quieter = _trial(
        script,
        name="quieter",
        i=1,
        d=16,
        error_m=0.006,
        tcp_span_m=0.0005,
    )

    assert script.select_best_trial((noisier, quieter)) is quieter
    assert quieter.accepted is True
    assert noisier.accepted is True


def test_trial_acceptance_requires_both_six_mm_error_and_two_mm_jitter():
    script = _load_script_module()

    assert _trial(
        script,
        name="good",
        i=0,
        d=0,
        error_m=0.006,
        tcp_span_m=0.002,
    ).accepted
    assert not _trial(
        script,
        name="large_error",
        i=0,
        d=0,
        error_m=0.0061,
    ).accepted
    assert not _trial(
        script,
        name="large_jitter",
        i=0,
        d=0,
        error_m=0.005,
        tcp_span_m=0.0021,
    ).accepted


def test_residual_correction_cancels_repeatable_joint_lag_once():
    script = _load_script_module()
    nominal = (0.0, 10.0, 20.0, 30.0, 40.0, 50.0)
    actual = (0.5, 9.0, 21.5, 29.75, 40.0, 49.5)

    corrected = script.calculate_residual_correction(nominal, actual)

    assert corrected == pytest.approx(
        (-0.5, 11.0, 18.5, 30.25, 40.0, 50.5)
    )


def test_residual_correction_rejects_more_than_two_degrees():
    script = _load_script_module()
    nominal = (0.0,) * 6
    actual = (0.0, 0.0, 2.01, 0.0, 0.0, 0.0)

    with pytest.raises(RuntimeError, match="超过一次补偿上限"):
        script.calculate_residual_correction(nominal, actual)


def test_integral_candidates_only_accept_a_tested_derivative_value():
    script = _load_script_module()

    with pytest.raises(ValueError, match="来自前三组"):
        script.integral_candidates(best_d=8)
