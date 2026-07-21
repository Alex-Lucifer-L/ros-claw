from threading import Event

import pytest

from rosclaw_mini.arm.base import ArmAdapter
from rosclaw_mini.arm.so100_plus_factory import (
    SO100_PLUS_MOTOR_NAMES,
    SO100PlusRobotConfig,
)
from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.execution.controller import ExecutionController
from rosclaw_mini.runtime import (
    ArmRuntime,
    build_mock_runtime,
    build_so100_plus_runtime,
)
from rosclaw_mini.skills.arm_skills import (
    build_so100_plus_right_follower_arm_skills,
)


class RecordingAdapter(ArmAdapter):
    def __init__(self, *, stop_event: Event | None = None) -> None:
        self.connected = True
        self.calls: list[str] = []
        self.stop_event = stop_event

    @property
    def is_connected(self) -> bool:
        return self.connected

    def connect(self) -> None:
        self.calls.append("connect")
        self.connected = True

    def disconnect(self) -> None:
        self.calls.append("disconnect")
        self.connected = False

    def move_to(self, x: float, y: float, z: float) -> None:
        self.calls.append("move_to")

    def open_gripper(self) -> None:
        self.calls.append("open_gripper")

    def close_gripper(self) -> None:
        self.calls.append("close_gripper")

    def stop(self) -> None:
        self.calls.append("stop")
        if self.stop_event is not None:
            self.stop_event.set()

    def disable_torque(self, *, emergency: bool = False) -> None:
        self.calls.append("disable_torque")


def _successful_result(command: Command) -> ExecutionResult:
    return ExecutionResult(
        command_id=command.command_id,
        skill_name=command.skill_name,
        success=True,
        message="完成",
    )


def test_mock_runtime_is_connected_and_uses_mock_workspace():
    runtime = build_mock_runtime(move_duration_seconds=0.0)

    assert runtime.adapter.is_connected is True
    assert runtime.skills["move_arm"].enabled is True
    assert runtime.skills["disable_torque"].enabled is False

    runtime.shutdown()

    assert runtime.adapter.is_connected is False


def test_runtime_shutdown_stops_waits_and_disconnects_without_disabling_torque():
    stop_event = Event()
    motion_started = Event()

    def runner(command: Command) -> ExecutionResult:
        motion_started.set()
        stop_event.wait(timeout=1.0)
        return _successful_result(command)

    adapter = RecordingAdapter(stop_event=stop_event)
    controller = ExecutionController(runner)
    runtime = ArmRuntime(adapter=adapter, skills={}, controller=controller)
    command = Command(
        command_id="move-001",
        skill_name="move_arm",
        params={"x": 0.1, "y": 0.2, "z": 0.3},
        source="user",
    )

    assert controller.submit(command) is True
    assert motion_started.wait(timeout=1.0) is True

    runtime.shutdown()

    assert adapter.calls == ["stop", "disconnect"]
    assert controller.is_running() is False
    assert adapter.is_connected is False

    # shutdown 可重复调用，但不会重复操作硬件。
    runtime.shutdown()
    assert adapter.calls == ["stop", "disconnect"]


def test_so100_plus_runtime_requires_risk_ack_before_factory_call():
    factory_called = False

    def forbidden_factory(config):
        nonlocal factory_called
        factory_called = True
        raise AssertionError("未确认风险时不应创建 Robot")

    config = SO100PlusRobotConfig(
        port="/path-not-accessed/robot",
        calibration_dir="/path-not-accessed/calibration",
        follower_name="right",
    )

    with pytest.raises(PermissionError, match="显式确认"):
        build_so100_plus_runtime(
            config,
            risk_acknowledged=False,
            robot_factory=forbidden_factory,
        )

    assert factory_called is False


def test_so100_plus_runtime_rejects_unregistered_follower_before_factory_call():
    factory_called = False

    def forbidden_factory(config):
        nonlocal factory_called
        factory_called = True
        raise AssertionError("错误 follower 不应创建 Robot")

    config = SO100PlusRobotConfig(
        port="/path-not-accessed/robot",
        calibration_dir="/path-not-accessed/calibration",
        follower_name="main",
    )

    with pytest.raises(ValueError, match="只登记给 follower 'right'"):
        build_so100_plus_runtime(
            config,
            risk_acknowledged=True,
            robot_factory=forbidden_factory,
        )

    assert factory_called is False


class FakeBus:
    motor_names = SO100_PLUS_MOTOR_NAMES

    def __init__(self) -> None:
        self.read_calls: list[str] = []

    def read(self, register: str):
        self.read_calls.append(register)
        return (10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0)


class FakeRobot:
    def __init__(self) -> None:
        self.is_connected = False
        self.follower_arms = {"right": FakeBus()}


class FakeKinematics:
    def __init__(
        self,
        current_tcp_position_m: tuple[float, float, float] = (
            0.35,
            0.0,
            0.22,
        ),
    ) -> None:
        self.driver_inputs: list[tuple[float, ...]] = []
        self.forward_inputs: list[tuple[float, ...]] = []
        self.current_tcp_position_m = current_tcp_position_m

    def driver_degrees_to_model_radians(self, values):
        values = tuple(values)
        self.driver_inputs.append(values)
        return tuple(value / 100.0 for value in values)

    def forward_position(self, values):
        values = tuple(values)
        self.forward_inputs.append(values)
        return self.current_tcp_position_m


class FakeSO100PlusAdapter(RecordingAdapter):
    def __init__(self, robot, gripper_config, **kwargs) -> None:
        super().__init__()
        self.robot = robot
        self.connected = robot.is_connected
        self.gripper_config = gripper_config
        self.kwargs = kwargs

    @property
    def is_connected(self) -> bool:
        return self.robot.is_connected

    def connect(self) -> None:
        self.calls.append("connect")
        self.robot.is_connected = True

    def disconnect(self) -> None:
        self.calls.append("disconnect")
        self.robot.is_connected = False


def test_so100_plus_runtime_reuses_factory_limits_and_right_follower_skills():
    config = SO100PlusRobotConfig(
        port="/path-not-accessed/robot",
        calibration_dir="/path-not-accessed/calibration",
        follower_name="right",
    )
    robot = FakeRobot()
    kinematics = FakeKinematics()
    adapters: list[FakeSO100PlusAdapter] = []
    motion_inputs: list[tuple[float, ...]] = []
    motion_limits = object()
    skills = None
    skill_adapter = None

    def robot_factory(received_config):
        assert received_config is config
        return robot

    def adapter_factory(*args, **kwargs):
        adapter = FakeSO100PlusAdapter(*args, **kwargs)
        adapters.append(adapter)
        return adapter

    def motion_limits_builder(current_joint_radians):
        motion_inputs.append(tuple(current_joint_radians))
        return motion_limits

    def skill_builder(adapter):
        nonlocal skill_adapter, skills
        skill_adapter = adapter
        skills = build_so100_plus_right_follower_arm_skills(adapter)
        return skills

    runtime = build_so100_plus_runtime(
        config,
        risk_acknowledged=True,
        robot_factory=robot_factory,
        kinematics_factory=lambda: kinematics,
        adapter_factory=adapter_factory,
        motion_limits_builder=motion_limits_builder,
        skill_builder=skill_builder,
    )

    assert len(adapters) == 2
    assert adapters[0].calls == ["connect"]
    assert runtime.adapter is adapters[1]
    assert skill_adapter is runtime.adapter
    assert runtime.skills is skills
    assert runtime.skills["move_arm"].enabled is True
    assert runtime.current_tcp_position_m == (0.35, 0.0, 0.22)
    assert runtime.move_arm_disabled_reason is None
    assert kinematics.driver_inputs == [
        (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
    ]
    assert kinematics.forward_inputs == [
        (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    ]
    assert motion_inputs == [(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)]
    assert adapters[1].kwargs["kinematics"] is kinematics
    assert adapters[1].kwargs["motion_limits"] is motion_limits
    assert adapters[1].kwargs["motion_config"] is not None
    assert robot.follower_arms["right"].read_calls == ["Present_Position"]

    runtime.shutdown()

    assert adapters[1].calls == ["stop", "disconnect"]


def test_so100_plus_runtime_disables_move_arm_when_startup_tcp_is_outside_workspace():
    config = SO100PlusRobotConfig(
        port="/path-not-accessed/robot",
        calibration_dir="/path-not-accessed/calibration",
        follower_name="right",
    )
    robot = FakeRobot()
    kinematics = FakeKinematics(
        current_tcp_position_m=(
            0.208819920,
            -0.021115885,
            0.002416365,
        )
    )
    adapters: list[FakeSO100PlusAdapter] = []

    def adapter_factory(*args, **kwargs):
        adapter = FakeSO100PlusAdapter(*args, **kwargs)
        adapters.append(adapter)
        return adapter

    runtime = build_so100_plus_runtime(
        config,
        risk_acknowledged=True,
        robot_factory=lambda _config: robot,
        kinematics_factory=lambda: kinematics,
        adapter_factory=adapter_factory,
        motion_limits_builder=lambda _current_joints: object(),
    )

    assert runtime.current_tcp_position_m == (
        0.208819920,
        -0.021115885,
        0.002416365,
    )
    assert runtime.skills["move_arm"].enabled is False
    assert runtime.skills["open_gripper"].enabled is True
    assert runtime.skills["close_gripper"].enabled is True
    assert runtime.skills["stop"].enabled is True
    assert runtime.move_arm_disabled_reason is not None
    assert "不在当前 right_follower 正式工作空间内" in (
        runtime.move_arm_disabled_reason
    )

    command = Command(
        command_id="outside-workspace-startup",
        skill_name="move_arm",
        params={"x": 0.35, "y": 0.0, "z": 0.22},
        source="user",
    )
    assert runtime.controller.submit(command) is True
    result = runtime.controller.wait(timeout=1.0)

    assert result is not None
    assert result.success is False
    assert result.message == "技能未启用: move_arm"
    assert "move_to" not in adapters[1].calls

    runtime.shutdown()

    assert adapters[1].calls == ["stop", "disconnect"]


def test_so100_plus_runtime_cleans_up_if_post_connect_assembly_fails():
    config = SO100PlusRobotConfig(
        port="/path-not-accessed/robot",
        calibration_dir="/path-not-accessed/calibration",
        follower_name="right",
    )
    robot = FakeRobot()
    adapters: list[FakeSO100PlusAdapter] = []

    def adapter_factory(*args, **kwargs):
        adapter = FakeSO100PlusAdapter(*args, **kwargs)
        adapters.append(adapter)
        return adapter

    def failing_limits_builder(current_joint_radians):
        raise RuntimeError("模拟 MotionLimits 装配失败")

    with pytest.raises(RuntimeError, match="模拟 MotionLimits 装配失败"):
        build_so100_plus_runtime(
            config,
            risk_acknowledged=True,
            robot_factory=lambda _config: robot,
            kinematics_factory=FakeKinematics,
            adapter_factory=adapter_factory,
            motion_limits_builder=failing_limits_builder,
        )

    assert adapters[0].calls == ["connect", "stop", "disconnect"]
    assert robot.is_connected is False
