from importlib.util import module_from_spec, spec_from_file_location
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from rosclaw_mini.arm.kinematics import (
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
)


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT / "scripts" / "check_so100_plus_candidate_workspace.py"
REPORT_PATH = (
    ROOT
    / "artifacts"
    / "so100_plus_rest_workspace"
    / "rest_workspace_report.json"
)


def _load_script_module():
    spec = spec_from_file_location(
        "check_so100_plus_candidate_workspace",
        SCRIPT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_real_report_builds_two_internal_points_and_return_path():
    script = _load_script_module()

    candidate = script.load_workspace_candidate(REPORT_PATH)
    checkpoints = script.build_validation_checkpoints(candidate)

    assert [checkpoint.name for checkpoint in checkpoints] == [
        "near_internal",
        "middle_internal",
        "near_internal_return",
        "initial_return",
    ]
    assert checkpoints[0].position_m == pytest.approx(
        (
            candidate.initial_position_m[0] + 0.03,
            candidate.initial_position_m[1],
            candidate.initial_position_m[2] + 0.03,
        )
    )
    assert checkpoints[1].position_m == pytest.approx(
        (
            candidate.initial_position_m[0] + 0.07,
            candidate.initial_position_m[1] - 0.01,
            candidate.initial_position_m[2] + 0.06,
        )
    )
    assert checkpoints[-1].position_m == pytest.approx(
        candidate.initial_position_m
    )
    for checkpoint in checkpoints:
        candidate.workspace_limits().validate_position(
            *checkpoint.position_m
        )
    assert (
        script.CANDIDATE_FINAL_JOINT_TOLERANCE_DEGREES
        == pytest.approx(3.0)
    )
    assert script.CANDIDATE_FINAL_TCP_TOLERANCE_M == pytest.approx(0.012)


def test_transition_only_mode_skips_all_candidate_internal_points():
    script = _load_script_module()
    candidate = script.load_workspace_candidate(REPORT_PATH)

    checkpoints = script.build_validation_checkpoints_for_mode(
        candidate,
        transition_only=True,
    )

    assert checkpoints == ()


def test_default_mode_keeps_candidate_internal_points():
    script = _load_script_module()
    candidate = script.load_workspace_candidate(REPORT_PATH)

    checkpoints = script.build_validation_checkpoints_for_mode(
        candidate,
        transition_only=False,
    )

    assert [checkpoint.name for checkpoint in checkpoints] == [
        "near_internal",
        "middle_internal",
        "near_internal_return",
        "initial_return",
    ]


def test_boundary_suite_combines_internal_six_faces_and_eight_corners():
    script = _load_script_module()
    candidate = script.load_workspace_candidate(REPORT_PATH)

    checkpoints = script.build_validation_checkpoints_for_mode(
        candidate,
        transition_only=False,
        boundary_suite=True,
    )
    boundary = script.build_boundary_validation_checkpoints(candidate)

    assert len(checkpoints) == 19
    assert checkpoints[:4] == script.build_validation_checkpoints(candidate)
    assert checkpoints[4:] == boundary
    assert len(boundary) == 15
    assert sum("boundary_face_" in item.name for item in boundary) == 6
    assert sum("boundary_corner_" in item.name for item in boundary) == 8
    assert boundary[-1].name == "boundary_initial_return"
    assert boundary[-1].position_m == pytest.approx(
        candidate.initial_position_m
    )

    margin = (
        candidate.grid_step_m * script.BOUNDARY_MARGIN_GRID_STEPS
    )
    non_return_points = boundary[:-1]
    for checkpoint in non_return_points:
        candidate.workspace_limits().validate_position(
            *checkpoint.position_m
        )
        for axis, value in enumerate(checkpoint.position_m):
            assert (
                candidate.lower_m[axis] + margin
                <= value
                <= candidate.upper_m[axis] - margin
            )


def test_boundary_resume_is_empty_after_all_points_have_results():
    script = _load_script_module()
    candidate = script.load_workspace_candidate(REPORT_PATH)

    checkpoints = script.build_validation_checkpoints_for_mode(
        candidate,
        transition_only=False,
        boundary_resume=True,
    )

    assert checkpoints == ()


def test_parser_accepts_transition_only_mode():
    script = _load_script_module()

    args = script.build_parser().parse_args(
        [
            "--port",
            "/dev/lerobot_right",
            "--calibration-dir",
            "/tmp/calibration",
            "--follower-name",
            "right",
            "--transition-only",
        ]
    )

    assert args.transition_only is True


def test_parser_accepts_boundary_suite_and_rejects_conflicting_scope():
    script = _load_script_module()
    common = [
        "--port",
        "/dev/lerobot_right",
        "--calibration-dir",
        "/tmp/calibration",
        "--follower-name",
        "right",
    ]

    args = script.build_parser().parse_args(common + ["--boundary-suite"])

    assert args.boundary_suite is True
    resume_args = script.build_parser().parse_args(
        common
        + [
            "--boundary-resume",
            "--continue-on-convergence-error",
        ]
    )
    assert resume_args.boundary_resume is True
    assert resume_args.continue_on_convergence_error is True
    with pytest.raises(SystemExit):
        script.build_parser().parse_args(
            common + ["--transition-only", "--boundary-suite"]
        )
    with pytest.raises(SystemExit):
        script.build_parser().parse_args(
            common + ["--boundary-suite", "--boundary-resume"]
        )


def test_only_explicit_final_convergence_error_can_continue():
    script = _load_script_module()
    convergence_error = script.SO100PlusMotionConvergenceError(
        "最终 TCP 超差"
    )

    assert script.can_continue_after_checkpoint_error(
        convergence_error,
        enabled=True,
    )
    assert not script.can_continue_after_checkpoint_error(
        convergence_error,
        enabled=False,
    )
    assert not script.can_continue_after_checkpoint_error(
        RuntimeError("串口故障"),
        enabled=True,
    )


def test_complete_boundary_suite_has_offline_ik_and_collision_free_paths():
    script = _load_script_module()
    candidate = script.load_workspace_candidate(REPORT_PATH)
    kinematics = script.SO100PlusKinematics()
    storage_joints = kinematics.driver_degrees_to_model_radians(
        script.EXPECTED_STORAGE_REST_DRIVER_DEGREES
    )
    transition = script.validate_storage_to_initial_transition(
        storage_joints,
        kinematics,
    )
    joint_limits = (
        script.build_so100_plus_right_follower_execution_joint_limits(
            storage_joints,
            max_step_radians=script.MAX_JOINT_STEP_RADIANS,
        )
    )
    checkpoints = script.build_validation_checkpoints_for_mode(
        candidate,
        transition_only=False,
        boundary_suite=True,
    )

    validation = script.validate_checkpoint_suite_offline(
        checkpoints,
        initial_joint_radians=transition.target_joint_radians,
        candidate=candidate,
        kinematics=kinematics,
        joint_limits=joint_limits,
    )

    assert validation[0] == 19
    assert validation[1] > 0
    assert validation[2] > 0
    assert validation[3] >= 0


def test_report_without_all_neighbor_paths_is_rejected(tmp_path):
    script = _load_script_module()
    payload = json.loads(REPORT_PATH.read_text(encoding="utf-8"))
    payload["box_directed_neighbor_check"]["all_valid"] = False
    path = tmp_path / "invalid_report.json"
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="相邻网格路径"):
        script.load_workspace_candidate(path)


def test_storage_rest_start_accepts_measured_pose_and_rejects_large_error():
    script = _load_script_module()
    good_snapshot = script.PoseSnapshot(
        driver_degrees=script.EXPECTED_STORAGE_REST_DRIVER_DEGREES,
        joint_radians=(0.0,) * 6,
        tcp_position_m=(0.0, 0.0, 0.0),
        torque_enabled=(0,) * 7,
    )

    max_joint_error = script.validate_storage_rest_start(good_snapshot)

    assert max_joint_error == pytest.approx(0.0)

    bad_driver = list(script.EXPECTED_STORAGE_REST_DRIVER_DEGREES)
    bad_driver[2] += 10.0
    bad_snapshot = script.PoseSnapshot(
        driver_degrees=tuple(bad_driver),
        joint_radians=(0.0,) * 6,
        tcp_position_m=(0.0, 0.0, 0.0),
        torque_enabled=(0,) * 7,
    )
    with pytest.raises(RuntimeError, match="不是本机已测的 follower_rest"):
        script.validate_storage_rest_start(bad_snapshot)


def test_rest_start_rejects_enabled_torque_before_pose_checks():
    script = _load_script_module()
    snapshot = script.PoseSnapshot(
        driver_degrees=script.EXPECTED_STORAGE_REST_DRIVER_DEGREES,
        joint_radians=(0.0,) * 6,
        tcp_position_m=(0.0, 0.0, 0.0),
        torque_enabled=(0, 0, 1, 0, 0, 0, 0),
    )

    with pytest.raises(RuntimeError, match="力矩仍开启"):
        script.validate_storage_rest_start(snapshot)


def test_initial_pose_validation_is_separate_from_storage_rest():
    script = _load_script_module()
    candidate = script.load_workspace_candidate(REPORT_PATH)
    snapshot = script.PoseSnapshot(
        driver_degrees=(0.0,) * 6,
        joint_radians=SO100_PLUS_JOYCON_INITIAL_RADIANS,
        tcp_position_m=candidate.initial_position_m,
        torque_enabled=(1,) * 7,
    )

    joint_error, tcp_error = script.validate_initial_pose(
        snapshot,
        candidate,
    )

    assert joint_error == pytest.approx(0.0)
    assert tcp_error == pytest.approx(0.0)


def test_measured_storage_to_initial_transition_only_clears_contacts():
    script = _load_script_module()
    kinematics = script.SO100PlusKinematics()
    start_joints = kinematics.driver_degrees_to_model_radians(
        script.EXPECTED_STORAGE_REST_DRIVER_DEGREES
    )

    transition = script.validate_storage_to_initial_transition(
        start_joints,
        kinematics,
    )

    assert transition.initial_contact_pair_count == 7
    assert transition.last_contact_step >= 0
    assert transition.max_joint_change_degrees == pytest.approx(
        71.719,
        abs=0.01,
    )
    assert transition.cartesian_distance_m == pytest.approx(
        0.21109,
        abs=1e-4,
    )
    assert transition.target_joint_radians == pytest.approx(
        SO100_PLUS_JOYCON_INITIAL_RADIANS
    )


def test_recovery_returns_to_initial_then_storage_rest(
    monkeypatch,
    tmp_path,
):
    script = _load_script_module()
    candidate = script.load_workspace_candidate(REPORT_PATH)
    storage_joints = (0.1,) * 6
    transition = script.TransitionValidation(
        escape_joint_radians=(0.2,) * 6,
        target_joint_radians=SO100_PLUS_JOYCON_INITIAL_RADIANS,
        path_positions_m=((0.1, 0.0, 0.1),),
        max_joint_change_degrees=1.0,
        cartesian_distance_m=0.1,
        cartesian_path_length_m=0.1,
        initial_contact_pair_count=0,
        last_contact_step=-1,
    )
    calls = []

    class CandidateAdapter:
        def plan_move_to(self, *position):
            calls.append(("plan_initial", position))
            return SimpleNamespace(
                current_joint_radians=(0.0,) * 6,
                target_joint_radians=(0.1,) * 6,
            )

        def move_to(self, *position):
            calls.append(("move_initial", position))

    monkeypatch.setattr(
        script,
        "validate_collision_free_joint_path",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        script,
        "_record_settle_report",
        lambda **_kwargs: None,
    )
    monkeypatch.setattr(
        script,
        "_record_checkpoint",
        lambda **_kwargs: 0.0,
    )
    monkeypatch.setattr(
        script,
        "_execute_joint_checkpoint",
        lambda **kwargs: calls.append(
            (
                kwargs["name"],
                tuple(kwargs["target_joint_radians"]),
            )
        ),
    )
    monkeypatch.setattr(
        script,
        "_read_pose_snapshot",
        lambda *_args: SimpleNamespace(),
    )
    monkeypatch.setattr(
        script,
        "validate_storage_rest_start",
        lambda *_args, **_kwargs: 0.25,
    )

    max_error = script._return_to_storage_rest(
        candidate_adapter=CandidateAdapter(),
        transition_adapter=object(),
        candidate=candidate,
        transition=transition,
        storage_joint_radians=storage_joints,
        path=tmp_path / "recovery.jsonl",
        follower_bus=object(),
        kinematics=object(),
        return_from_candidate=True,
    )

    assert max_error == pytest.approx(0.25)
    assert calls == [
        ("plan_initial", candidate.initial_position_m),
        ("move_initial", candidate.initial_position_m),
        ("storage_escape_return", transition.escape_joint_radians),
        ("follower_rest_return", storage_joints),
    ]


def test_candidate_path_is_rechecked_from_its_actual_joint_start():
    script = _load_script_module()
    kinematics = script.SO100PlusKinematics()
    candidate = script.load_workspace_candidate(REPORT_PATH)
    checkpoint = script.build_validation_checkpoints(candidate)[0]
    start = SO100_PLUS_JOYCON_INITIAL_RADIANS
    joint_limits = (
        script.build_so100_plus_right_follower_execution_joint_limits(
            start,
            max_step_radians=script.MAX_JOINT_STEP_RADIANS,
        )
    )
    target = kinematics.solve_position(
        start,
        checkpoint.position_m,
        joint_limits=joint_limits,
    )

    validation = script.validate_collision_free_joint_path(
        start,
        target,
        kinematics,
    )

    assert validation.step_count > 0
    assert validation.max_cartesian_step_m > 0
    assert validation.minimum_tcp_z_m >= 0


def test_settle_report_is_written_as_structured_json(tmp_path):
    script = _load_script_module()
    output_path = tmp_path / "settle.jsonl"
    report = SimpleNamespace(
        motor_names=("joint_a", "joint_b"),
        position_samples_degrees=((1.0, 2.0), (1.1, 1.9)),
        position_span_degrees=(0.1, 0.1),
        tcp_samples_m=((0.1, 0.2, 0.3), (0.101, 0.199, 0.3)),
        tcp_min_m=(0.1, 0.199, 0.3),
        tcp_max_m=(0.101, 0.2, 0.3),
        tcp_mean_m=(0.1005, 0.1995, 0.3),
        duration_seconds=0.75,
    )

    script._record_settle_report(
        path=output_path,
        checkpoint_name="near_internal",
        adapter=SimpleNamespace(last_settle_report=report),
    )

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["type"] == "settle_summary"
    assert payload["checkpoint"] == "near_internal"
    assert payload["position_span_degrees"] == pytest.approx([0.1, 0.1])
    assert payload["tcp_mean_m"] == pytest.approx([0.1005, 0.1995, 0.3])
