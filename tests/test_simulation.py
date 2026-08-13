"""Headless tests for the explicitly simulation-only SO-100 research backend."""

from __future__ import annotations

from pathlib import Path

import pytest

from rosclaw_mini.arm.so100_plus_session import ArmSessionState
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.runtime import build_sim_runtime
from rosclaw_mini.main import build_parser, build_runtime_from_args
from rosclaw_mini.simulation.benchmark import (
    run_headless_benchmark,
    save_benchmark_results,
    summarize_runs,
)
from rosclaw_mini.simulation.calibration import SimulationEyeToHandCalibration
from rosclaw_mini.simulation.camera import VirtualRGBDCamera
from rosclaw_mini.simulation.config import (
    SimulationConfigurationError,
    load_simulation_camera_config,
)
from rosclaw_mini.simulation.perception import SimulatedColorVLM
from rosclaw_mini.simulation.pipeline import SimulationGraspPipeline
from rosclaw_mini.simulation.strategy import GraspStrategyVersion
from rosclaw_mini.simulation.adapter import SimulationMotionStoppedError
from rosclaw_mini.vision.localization import RealSenseLocalizationService


def _command(skill_name: str, params: dict) -> Command:
    return Command("sim-test", skill_name, params, "test")


def test_sim_runtime_uses_isolated_adapter_and_work_session():
    runtime = build_sim_runtime()
    try:
        assert runtime.adapter.__class__.__name__ == "SimulatedArmAdapter"
        assert runtime.adapter.is_connected is True
        assert runtime.session_state is ArmSessionState.WORK
        assert runtime.adapter.world.scene.simulation_only is True
        assert "/dev" not in runtime.adapter.__class__.__module__
    finally:
        runtime.shutdown()


def test_main_sim_backend_builds_without_real_port_or_risk_acknowledgement():
    args = build_parser().parse_args(["--backend", "sim", "--sim-scene", "standard"])
    runtime = build_runtime_from_args(args)
    try:
        assert runtime.adapter.__class__.__name__ == "SimulatedArmAdapter"
        assert runtime.session_state is ArmSessionState.WORK
    finally:
        runtime.shutdown()


def test_sim_adapter_executes_the_frozen_prevalidated_waypoints():
    runtime = build_sim_runtime()
    try:
        target = runtime.adapter.world.reference_tcp()
        target = (target[0], target[1], target[2] + 0.08)
        plan = runtime.adapter.plan_move_to(*target)
        runtime.adapter.move_to(*target)

        assert plan.is_final_execution_plan is True
        assert runtime.adapter.last_motion_plan is not None
        assert tuple(runtime.adapter.executed_waypoints) == plan.waypoints_radians
        assert runtime.adapter.world.motion_records[-1].collision_checked is True
    finally:
        runtime.shutdown()


def test_sim_rejects_workspace_target_before_any_waypoint_write():
    runtime = build_sim_runtime(scene_name="boundary_reject")
    try:
        result = run_command(
            _command("move_arm", {"x": 0.60, "y": 0.0, "z": 0.24}),
            runtime.skills,
        )
        assert result.success is False
        assert "UnsafeCommand" in result.message
        assert runtime.adapter.executed_waypoints == []
    finally:
        runtime.shutdown()


def test_sim_stop_before_first_waypoint_writes_nothing():
    runtime = build_sim_runtime()
    try:
        target = runtime.adapter.world.reference_tcp()
        runtime.adapter.waypoint_hook = lambda index, plan: runtime.adapter.stop()
        with pytest.raises(SimulationMotionStoppedError):
            runtime.adapter.move_to(target[0], target[1], target[2] + 0.08)
        assert runtime.adapter.executed_waypoints == []
    finally:
        runtime.shutdown()


def test_sim_stop_after_middle_waypoint_stops_remaining_route_and_degrades_state():
    runtime = build_sim_runtime()
    try:
        def stop_after_two(index, plan):
            del plan
            if index == 2:
                runtime.controller.request_stop(_command("stop", {}))

        runtime.adapter.waypoint_hook = stop_after_two
        target = runtime.adapter.world.reference_tcp()
        command = _command(
            "move_arm",
            {"x": target[0], "y": target[1], "z": target[2] + 0.08},
        )
        assert runtime.controller.submit(command) is True
        result = runtime.controller.wait(timeout=5.0)
        assert result is not None and result.success is False
        assert len(runtime.adapter.executed_waypoints) == 2
        assert runtime.session_state is ArmSessionState.UNVERIFIED
    finally:
        runtime.shutdown()


def test_sim_rest_gate_unfolds_to_work_without_claiming_real_certification():
    runtime = build_sim_runtime(start_state=ArmSessionState.REST)
    try:
        blocked = run_command(
            _command("move_arm", {"x": 0.37, "y": -0.01, "z": 0.24}),
            runtime.skills,
        )
        assert blocked.success is False
        unfolded = run_command(_command("unfold_arm", {}), runtime.skills)
        assert unfolded.success is True
        assert runtime.session_state is ArmSessionState.WORK
        assert "不代表真机" in unfolded.message
    finally:
        runtime.shutdown()


def test_virtual_rgbd_frame_is_synchronized_and_localizes_from_pixels_only():
    runtime = build_sim_runtime()
    try:
        with VirtualRGBDCamera(runtime.adapter.world) as camera:
            frame = camera.capture_frame()
        assert frame.rgb.shape == (480, 640, 3)
        assert frame.aligned_depth.shape == (480, 640)
        assert frame.frame_number == 1
        assert frame.source.startswith("simulation:")
        assert (frame.aligned_depth > 0).any()

        service = RealSenseLocalizationService(
            client=SimulatedColorVLM(),
            camera_factory=lambda: VirtualRGBDCamera(runtime.adapter.world),
        )
        localized = service.locate("定位红色立方体")
        calibration = SimulationEyeToHandCalibration(runtime.adapter.world.scene.camera)
        base = calibration.transform_position_estimate(localized.position)
        truth = runtime.adapter.world.object_state("red_cube").grasp_point_m
        assert base.calibration_sha256.startswith("simulation:")
        assert sum((left - right) ** 2 for left, right in zip(base.base_point_m, truth)) ** 0.5 < 0.01
    finally:
        runtime.shutdown()


def test_simulation_camera_config_is_explicitly_isolated_from_real_calibration(tmp_path: Path):
    example = Path(__file__).resolve().parents[1] / "configs" / "simulation_camera.example.json"
    config = load_simulation_camera_config(example)
    assert config.simulation_only is True
    real_like = tmp_path / "real_like.json"
    real_like.write_text('{"schema_version": 1}', encoding="utf-8")
    with pytest.raises(SimulationConfigurationError, match="simulation_only"):
        load_simulation_camera_config(real_like)


def test_pipeline_runs_observe_plan_gateway_controller_lift_verify():
    runtime = build_sim_runtime()
    try:
        result = SimulationGraspPipeline(runtime, runtime.adapter.world).run(
            "抓取红色立方体",
            strategy_version=GraspStrategyVersion.HUMANLIKE_V3,
        )
        assert result.success is True
        assert result.plan is not None and result.preview is not None
        assert result.preview.is_safe is True
        assert result.stages == (
            "observe", "open_gripper", "pre_grasp", "orient", "approach",
            "close_gripper", "lift", "verify",
        )
        assert result.localization_error_m is not None
    finally:
        runtime.shutdown()


def test_stop_interrupts_simulated_grasp_before_following_steps():
    runtime = build_sim_runtime()
    try:
        def stop_before_first_write(index, plan):
            del plan
            if index == 0:
                runtime.controller.request_stop(_command("stop", {}))

        runtime.adapter.waypoint_hook = stop_before_first_write
        result = SimulationGraspPipeline(runtime, runtime.adapter.world).run("抓取红色立方体")
        assert result.success is False
        assert result.stages == ("observe", "open_gripper", "pre_grasp")
        assert "pre_grasp 失败" in result.message
        assert runtime.session_state is ArmSessionState.WORK
        assert runtime.adapter.world.gripper_is_open is True
    finally:
        runtime.shutdown()


def test_humanlike_strategy_reobserves_once_after_failed_verify(monkeypatch):
    runtime = build_sim_runtime()
    try:
        pipeline = SimulationGraspPipeline(runtime, runtime.adapter.world)
        observations = 0
        original_localize = pipeline._localize

        def recording_localize(task):
            nonlocal observations
            observations += 1
            return original_localize(task)

        monkeypatch.setattr(pipeline, "_localize", recording_localize)
        monkeypatch.setattr(
            runtime.adapter.world,
            "verify_lift",
            lambda *args, **kwargs: (False, "模拟滑落"),
        )
        result = pipeline.run(
            "抓取红色立方体",
            strategy_version=GraspStrategyVersion.HUMANLIKE_V3,
        )
        assert result.success is False
        assert result.reobserve_count == 1
        assert result.stages[-1] == "reobserve"
        assert observations == 2
    finally:
        runtime.shutdown()


def test_benchmark_records_matched_runs_and_compact_results(tmp_path: Path):
    runs = run_headless_benchmark(
        scenes=("standard", "boundary_reject"),
        seeds=(0,),
        strategies=(GraspStrategyVersion.BASELINE_V1, GraspStrategyVersion.EFFICIENT_V2),
    )
    assert len(runs) == 4
    assert any(run.success for run in runs)
    assert any(not run.success for run in runs)
    summaries = summarize_runs(runs)
    assert {item.strategy for item in summaries} == {"baseline_v1", "efficient_v2"}
    output = save_benchmark_results(tmp_path / "benchmark.json", runs)
    assert output.exists()
    assert output.with_suffix(".md").exists()
    assert '"simulation_only": true' in output.read_text(encoding="utf-8")


def test_versioned_prompt_files_document_safe_high_level_boundaries():
    root = Path(__file__).resolve().parents[1] / "prompts"
    for filename in (
        "grasp_baseline_v1.md",
        "grasp_efficient_v2.md",
        "grasp_humanlike_v3.md",
    ):
        content = (root / filename).read_text(encoding="utf-8")
        assert "target_object" in content
        assert "joint" in content.lower()
        assert "Safety" in content
