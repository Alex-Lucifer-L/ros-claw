import math
from pathlib import Path
from threading import Event

import pytest

from rosclaw_mini.arm.base import ArmAdapter
from rosclaw_mini.arm.kinematics import (
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
    SO100PlusKinematics,
)
from rosclaw_mini.arm.so100_plus import SO100_PLUS_REAL_HARDWARE_PROFILE
from rosclaw_mini.arm.so100_plus_session import (
    ArmSessionState,
    SO100_PLUS_MIDDLE_INTERNAL_RADIANS,
    SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M,
)
from rosclaw_mini.arm.so100_plus_trajectory_validation import (
    SO100PlusTrajectoryValidationUnavailableError,
)
from rosclaw_mini.arm.so100_plus_factory import (
    SO100_PLUS_MOTOR_NAMES,
    SO100PlusConfigurationError,
    SO100PlusRobotConfig,
)
from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.execution.controller import ExecutionController
from rosclaw_mini.runtime import (
    ArmRuntime,
    ArmRuntimeShutdownError,
    build_mock_runtime,
    build_so100_plus_runtime,
)
from rosclaw_mini.skills.arm_skills import (
    build_so100_plus_right_follower_arm_skills,
)


class RecordingAdapter(ArmAdapter):
    def __init__(
        self,
        *,
        stop_event: Event | None = None,
        stop_error: Exception | None = None,
        disconnect_on_stop: bool = False,
        disable_torque_error: Exception | None = None,
    ) -> None:
        self.connected = True
        self.calls: list[str] = []
        self.disconnect_event = Event()
        self.stop_event = stop_event
        self.stop_error = stop_error
        self.disconnect_on_stop = disconnect_on_stop
        self.disable_torque_error = disable_torque_error

    @property
    def is_connected(self) -> bool:
        return self.connected

    def connect(self) -> None:
        self.calls.append("connect")
        self.connected = True

    def disconnect(self) -> None:
        self.calls.append("disconnect")
        self.connected = False
        self.disconnect_event.set()

    def move_to(self, x: float, y: float, z: float) -> None:
        self.calls.append("move_to")

    def read_tcp_position(self) -> tuple[float, float, float]:
        self.calls.append("read_tcp_position")
        return (0.0, 0.0, 0.0)

    def open_gripper(self) -> None:
        self.calls.append("open_gripper")

    def close_gripper(self) -> None:
        self.calls.append("close_gripper")

    def stop(self) -> None:
        self.calls.append("stop")
        if self.stop_event is not None:
            self.stop_event.set()
        if self.disconnect_on_stop:
            self.connected = False
        if self.stop_error is not None:
            raise self.stop_error

    def disable_torque(self, *, emergency: bool = False) -> None:
        self.calls.append("disable_torque")
        if self.disable_torque_error is not None:
            raise self.disable_torque_error


def _successful_result(command: Command) -> ExecutionResult:
    return ExecutionResult(
        command_id=command.command_id,
        skill_name=command.skill_name,
        success=True,
        message="完成",
    )


def _certified_robot_config(tmp_path: Path) -> SO100PlusRobotConfig:
    repository_root = Path(__file__).resolve().parents[1]
    source = (
        repository_root
        / "lerobot-joycon_plus"
        / ".cache"
        / "calibration"
        / "so100_plus"
        / "right_follower.json"
    )
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    (calibration_dir / "right_follower.json").write_bytes(source.read_bytes())
    return SO100PlusRobotConfig(
        port="/dev/lerobot_right",
        calibration_dir=calibration_dir,
        follower_name="right",
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


def test_runtime_shutdown_from_rest_disables_torque_before_disconnect():
    adapter = RecordingAdapter()

    class RestSession:
        state = ArmSessionState.REST

        def __init__(self):
            self.calls = []

        def request_stop(self):
            self.calls.append("request_stop")

    session = RestSession()
    runtime = ArmRuntime(
        adapter=adapter,
        skills={},
        controller=ExecutionController(_successful_result),
        session=session,
    )

    runtime.shutdown()

    assert session.calls == ["request_stop"]
    assert adapter.calls == ["disable_torque", "disconnect"]
    assert runtime.torque_disabled_on_shutdown is True
    assert adapter.is_connected is False


def test_runtime_shutdown_from_rest_disconnects_and_reports_torque_failure():
    adapter = RecordingAdapter(
        disable_torque_error=RuntimeError("模拟 follower_rest 复核失败"),
    )

    class RestSession:
        state = ArmSessionState.REST

        def request_stop(self):
            return None

    runtime = ArmRuntime(
        adapter=adapter,
        skills={},
        controller=ExecutionController(_successful_result),
        session=RestSession(),
    )

    with pytest.raises(
        ArmRuntimeShutdownError,
        match="REST 状态关闭力矩失败.*follower_rest 复核失败",
    ):
        runtime.shutdown()

    assert adapter.calls == ["disable_torque", "disconnect"]
    assert runtime.torque_disabled_on_shutdown is False
    assert adapter.is_connected is False


def test_deferred_shutdown_disables_torque_if_session_finishes_in_rest():
    motion_started = Event()
    allow_motion_finish = Event()

    def runner(command: Command) -> ExecutionResult:
        motion_started.set()
        allow_motion_finish.wait(timeout=1.0)
        return _successful_result(command)

    adapter = RecordingAdapter()
    controller = ExecutionController(runner)

    class RestSession:
        state = ArmSessionState.REST

        def request_stop(self):
            return None

    runtime = ArmRuntime(
        adapter=adapter,
        skills={},
        controller=controller,
        session=RestSession(),
        shutdown_wait_timeout_seconds=0.01,
    )
    command = Command(
        command_id="deferred-rest-shutdown",
        skill_name="query",
        params={},
        source="user",
    )
    assert controller.submit(command) is True
    assert motion_started.wait(timeout=1.0) is True

    with pytest.raises(ArmRuntimeShutdownError, match="未结束"):
        runtime.shutdown()

    assert adapter.calls == []
    allow_motion_finish.set()
    assert adapter.disconnect_event.wait(timeout=1.0) is True
    runtime.shutdown()

    assert adapter.calls == ["disable_torque", "disconnect"]
    assert runtime.torque_disabled_on_shutdown is True


def test_runtime_shutdown_waits_and_disconnects_after_stop_raises():
    stop_event = Event()
    motion_started = Event()
    allow_motion_finish = Event()
    wait_timeouts: list[float | None] = []

    def runner(command: Command) -> ExecutionResult:
        motion_started.set()
        stop_event.wait(timeout=1.0)
        allow_motion_finish.wait(timeout=1.0)
        return _successful_result(command)

    adapter = RecordingAdapter(
        stop_event=stop_event,
        stop_error=RuntimeError("模拟 stop 写入失败"),
    )
    controller = ExecutionController(runner)
    original_wait = controller.wait

    def recording_wait(timeout=None):
        wait_timeouts.append(timeout)
        allow_motion_finish.set()
        return original_wait(timeout=timeout)

    controller.wait = recording_wait
    runtime = ArmRuntime(
        adapter=adapter,
        skills={},
        controller=controller,
        shutdown_wait_timeout_seconds=0.5,
    )
    command = Command(
        command_id="stop-error",
        skill_name="move_arm",
        params={"x": 0.1, "y": 0.2, "z": 0.3},
        source="user",
    )

    assert controller.submit(command) is True
    assert motion_started.wait(timeout=1.0) is True

    with pytest.raises(ArmRuntimeShutdownError, match="stop 失败"):
        runtime.shutdown()

    assert controller.is_running() is False
    assert wait_timeouts == [0.5]
    assert adapter.calls == ["stop", "disconnect"]
    assert adapter.is_connected is False
    assert "disable_torque" not in adapter.calls


def test_runtime_shutdown_timeout_keeps_adapter_connected_until_worker_finishes():
    motion_started = Event()
    allow_motion_finish = Event()

    def runner(command: Command) -> ExecutionResult:
        motion_started.set()
        allow_motion_finish.wait(timeout=1.0)
        return _successful_result(command)

    adapter = RecordingAdapter()
    controller = ExecutionController(runner)
    runtime = ArmRuntime(
        adapter=adapter,
        skills={},
        controller=controller,
        shutdown_wait_timeout_seconds=0.01,
    )
    command = Command(
        command_id="shutdown-timeout",
        skill_name="move_arm",
        params={"x": 0.1, "y": 0.2, "z": 0.3},
        source="user",
    )

    assert controller.submit(command) is True
    assert motion_started.wait(timeout=1.0) is True

    with pytest.raises(ArmRuntimeShutdownError, match="未结束"):
        runtime.shutdown()

    assert controller.is_running() is True
    assert adapter.is_connected is True
    assert adapter.calls == ["stop"]
    assert "disconnect" not in adapter.calls
    assert "disable_torque" not in adapter.calls

    allow_motion_finish.set()
    assert adapter.disconnect_event.wait(timeout=1.0) is True
    runtime.shutdown()

    assert controller.is_running() is False
    assert adapter.calls == ["stop", "disconnect"]
    assert adapter.is_connected is False


def test_runtime_shutdown_reports_adapter_disconnect_during_stop():
    stop_event = Event()
    motion_started = Event()

    def runner(command: Command) -> ExecutionResult:
        motion_started.set()
        stop_event.wait(timeout=1.0)
        return _successful_result(command)

    adapter = RecordingAdapter(
        stop_event=stop_event,
        disconnect_on_stop=True,
    )
    controller = ExecutionController(runner)
    runtime = ArmRuntime(
        adapter=adapter,
        skills={},
        controller=controller,
        shutdown_wait_timeout_seconds=0.5,
    )
    command = Command(
        command_id="unexpected-disconnect",
        skill_name="move_arm",
        params={"x": 0.1, "y": 0.2, "z": 0.3},
        source="user",
    )

    assert controller.submit(command) is True
    assert motion_started.wait(timeout=1.0) is True

    with pytest.raises(ArmRuntimeShutdownError, match="意外断开"):
        runtime.shutdown()

    assert controller.is_running() is False
    assert adapter.is_connected is False
    assert adapter.calls == ["stop"]
    assert "disable_torque" not in adapter.calls


def test_runtime_exit_from_work_only_stops_and_disconnects_without_folding():
    adapter = RecordingAdapter()

    class WorkSession:
        state = ArmSessionState.WORK
        state_reason = "测试中的 WORK"

        def __init__(self):
            self.calls = []

        def request_stop(self):
            assert adapter.is_connected is True
            self.calls.append("request_stop")

    session = WorkSession()
    runtime = ArmRuntime(
        adapter=adapter,
        skills={},
        controller=ExecutionController(_successful_result),
        session=session,
    )

    assert "未处于认证 follower_rest" in runtime.exit_pose_warning

    runtime.shutdown()

    assert session.calls == ["request_stop"]
    assert adapter.calls == ["disconnect"]
    assert adapter.is_connected is False
    assert "disable_torque" not in adapter.calls


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


def test_irregular_workspace_failure_happens_before_robot_factory(tmp_path):
    config = _certified_robot_config(tmp_path)
    robot_factory_called = False

    def forbidden_robot_factory(_config):
        nonlocal robot_factory_called
        robot_factory_called = True
        raise AssertionError("网格失败时不应创建 Robot")

    def unavailable_workspace():
        raise RuntimeError("模拟不规则工作空间网格不可用")

    with pytest.raises(RuntimeError, match="网格不可用"):
        build_so100_plus_runtime(
            config,
            risk_acknowledged=True,
            robot_factory=forbidden_robot_factory,
            irregular_workspace_factory=unavailable_workspace,
        )

    assert robot_factory_called is False


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

    def __init__(self, positions=None) -> None:
        self.read_calls: list[str] = []
        self.positions = (
            (10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0)
            if positions is None
            else tuple(positions)
        )

    def read(self, register: str):
        self.read_calls.append(register)
        return self.positions


class FakeRobot:
    def __init__(self, positions=None) -> None:
        self.is_connected = False
        self.follower_arms = {"right": FakeBus(positions)}


class FakeKinematics:
    def __init__(
        self,
        current_tcp_position_m: tuple[float, float, float] = (
            SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M
        ),
        current_joint_radians: tuple[float, ...] = (
            SO100_PLUS_MIDDLE_INTERNAL_RADIANS
        ),
    ) -> None:
        self.driver_inputs: list[tuple[float, ...]] = []
        self.forward_inputs: list[tuple[float, ...]] = []
        self.current_tcp_position_m = current_tcp_position_m
        self.current_joint_radians = current_joint_radians

    def driver_degrees_to_model_radians(self, values):
        values = tuple(values)
        self.driver_inputs.append(values)
        return self.current_joint_radians

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

    def move_joints(self, joint_radians) -> None:
        self.calls.append(("move_joints", tuple(joint_radians)))


class FakeTrajectoryValidator:
    """运行时装配测试不加载 MuJoCo，也不访问任何设备。"""


def _fake_trajectory_validator_factory():
    return FakeTrajectoryValidator()


def test_so100_plus_runtime_reuses_factory_limits_and_right_follower_skills(
    tmp_path,
):
    config = _certified_robot_config(tmp_path)
    robot = FakeRobot()
    kinematics = FakeKinematics()
    adapters: list[FakeSO100PlusAdapter] = []
    motion_inputs: list[tuple[float, ...]] = []
    motion_workspaces = []
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

    def motion_limits_builder(current_joint_radians, *, workspace):
        motion_inputs.append(tuple(current_joint_radians))
        motion_workspaces.append(workspace)
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
        trajectory_validator_factory=_fake_trajectory_validator_factory,
        skill_builder=skill_builder,
    )

    assert len(adapters) == 3
    assert adapters[0].calls == ["connect"]
    assert runtime.adapter is adapters[1]
    assert skill_adapter is runtime.adapter
    assert runtime.skills is not skills
    assert runtime.skills["move_arm"].enabled is True
    assert {"unfold_arm", "fold_arm", "revalidate_state"} <= (
        runtime.skills.keys()
    )
    assert runtime.skills["revalidate_state"].risk_level == "low"
    assert runtime.skills["revalidate_state"].params_schema == {}
    assert runtime.session_state is ArmSessionState.WORK
    assert runtime.current_tcp_position_m == (
        SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M
    )
    assert runtime.move_arm_disabled_reason is None
    assert kinematics.driver_inputs == [
        (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
    ]
    assert SO100_PLUS_JOYCON_INITIAL_RADIANS in kinematics.forward_inputs
    assert motion_inputs == [SO100_PLUS_MIDDLE_INTERNAL_RADIANS]
    assert motion_workspaces == [runtime.session.work_planning_envelope]
    assert runtime.skills["move_arm"].params_schema["x"].max_value == (
        runtime.session.work_workspace_aabb.x.maximum
    )
    assert runtime.skills["move_arm"].params_schema["y"].min_value == (
        runtime.session.work_workspace_aabb.y.minimum
    )
    assert adapters[1].kwargs["kinematics"] is kinematics
    assert adapters[1].kwargs["motion_limits"] is motion_limits
    assert adapters[1].kwargs["motion_config"] is not None
    assert adapters[2].kwargs["kinematics"] is kinematics
    assert adapters[2].kwargs["motion_config"] is not None
    assert robot.follower_arms["right"].read_calls == ["Present_Position"]

    runtime.shutdown()

    assert adapters[1].calls == ["stop", "disconnect"]
    assert adapters[2].calls == []


def test_so100_plus_runtime_recognizes_storage_feedback_as_rest(tmp_path):
    config = _certified_robot_config(tmp_path)
    storage_driver_degrees = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_driver_degrees
    )
    storage_joint_radians = (
        SO100PlusKinematics.driver_degrees_to_model_radians(
            storage_driver_degrees
        )
    )
    robot = FakeRobot((*storage_driver_degrees, 0.0))
    kinematics = FakeKinematics(
        current_tcp_position_m=(0.183792, -0.049054, 0.004970),
        current_joint_radians=storage_joint_radians,
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
        trajectory_validator_factory=_fake_trajectory_validator_factory,
    )

    assert runtime.session_state is ArmSessionState.REST
    assert runtime.move_arm_disabled_reason is None
    assert {"unfold_arm", "fold_arm", "revalidate_state"} <= (
        runtime.skills.keys()
    )

    command = Command(
        command_id="rest-rejects-move",
        skill_name="move_arm",
        params={"x": 0.35, "y": 0.0, "z": 0.22},
        source="user",
    )
    assert runtime.controller.submit(command) is True
    result = runtime.controller.wait(timeout=1.0)

    assert result is not None
    assert result.success is False
    assert "当前状态为 REST" in result.message
    assert "move_to" not in adapters[1].calls

    runtime.shutdown()

    assert adapters[2].calls == ["stop"]
    assert adapters[1].calls == ["disable_torque", "disconnect"]
    assert runtime.torque_disabled_on_shutdown is True


def test_so100_plus_runtime_disables_move_arm_when_startup_tcp_is_outside_workspace(
    tmp_path,
):
    config = _certified_robot_config(tmp_path)
    robot = FakeRobot()
    kinematics = FakeKinematics(
        current_tcp_position_m=(
            0.208819920,
            -0.021115885,
            0.002416365,
        ),
        current_joint_radians=tuple(
            value + (math.radians(10.0) if index == 2 else 0.0)
            for index, value in enumerate(
                SO100_PLUS_JOYCON_INITIAL_RADIANS
            )
        ),
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
        trajectory_validator_factory=_fake_trajectory_validator_factory,
    )

    assert runtime.current_tcp_position_m == (
        0.208819920,
        -0.021115885,
        0.002416365,
    )
    assert runtime.skills["move_arm"].enabled is True
    assert runtime.session_state is ArmSessionState.UNVERIFIED
    assert runtime.skills["open_gripper"].enabled is True
    assert runtime.skills["close_gripper"].enabled is True
    assert runtime.skills["stop"].enabled is True
    assert runtime.move_arm_disabled_reason is not None
    assert "当前姿态无法认证" in (
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
    assert "当前状态为 UNVERIFIED" in result.message
    assert "move_to" not in adapters[1].calls

    runtime.shutdown()

    assert adapters[1].calls == ["stop", "disconnect"]


def test_so100_plus_runtime_disables_move_arm_when_tcp_inside_but_pose_uncertified(
    tmp_path,
):
    config = _certified_robot_config(tmp_path)
    robot = FakeRobot()
    current_joints = list(SO100_PLUS_MIDDLE_INTERNAL_RADIANS)
    current_joints[2] += math.radians(6.0)
    kinematics = FakeKinematics(
        current_tcp_position_m=SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M,
        current_joint_radians=tuple(current_joints),
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
        trajectory_validator_factory=_fake_trajectory_validator_factory,
    )

    assert runtime.skills["move_arm"].enabled is True
    assert runtime.session_state is ArmSessionState.UNVERIFIED
    assert runtime.move_arm_disabled_reason is not None
    assert "middle_internal WORK 姿态" in (
        runtime.move_arm_disabled_reason
    )
    assert "ellbow_joint" in runtime.move_arm_disabled_reason

    command = Command(
        command_id="uncertified-pose",
        skill_name="move_arm",
        params={"x": 0.35, "y": 0.0, "z": 0.22},
        source="user",
    )
    assert runtime.controller.submit(command) is True
    result = runtime.controller.wait(timeout=1.0)

    assert result is not None
    assert result.success is False
    assert "当前状态为 UNVERIFIED" in result.message
    assert "move_to" not in adapters[1].calls

    runtime.shutdown()


@pytest.mark.parametrize(
    "full_turn_offset",
    (2 * math.pi, -2 * math.pi),
    ids=("plus_2pi", "minus_2pi"),
)
def test_so100_plus_runtime_rejects_full_turn_joint_aliases(
    tmp_path,
    full_turn_offset,
):
    config = _certified_robot_config(tmp_path)
    robot = FakeRobot()
    current_joints = list(SO100_PLUS_MIDDLE_INTERNAL_RADIANS)
    current_joints[5] += full_turn_offset
    kinematics = FakeKinematics(
        current_tcp_position_m=SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M,
        current_joint_radians=tuple(current_joints),
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
        trajectory_validator_factory=_fake_trajectory_validator_factory,
    )

    assert runtime.skills["move_arm"].enabled is True
    assert runtime.session_state is ArmSessionState.UNVERIFIED
    assert runtime.move_arm_disabled_reason is not None
    assert "wrist_roll_joint" in runtime.move_arm_disabled_reason
    assert "360.000°" in runtime.move_arm_disabled_reason
    assert "move_to" not in adapters[1].calls

    runtime.shutdown()


def test_so100_plus_runtime_rejects_uncertified_port_before_factory(
    tmp_path,
):
    certified_config = _certified_robot_config(tmp_path)
    config = SO100PlusRobotConfig(
        port="/dev/ttyACM0",
        calibration_dir=certified_config.calibration_dir,
        follower_name=certified_config.follower_name,
    )
    factory_called = False

    def forbidden_factory(_config):
        nonlocal factory_called
        factory_called = True
        raise AssertionError("端口不匹配时不应创建 Robot")

    with pytest.raises(
        SO100PlusConfigurationError,
        match="/dev/lerobot_right",
    ):
        build_so100_plus_runtime(
            config,
            risk_acknowledged=True,
            robot_factory=forbidden_factory,
        )

    assert factory_called is False


def test_so100_plus_runtime_rejects_changed_calibration_before_factory(
    tmp_path,
):
    config = _certified_robot_config(tmp_path)
    config.calibration_path.write_bytes(
        config.calibration_path.read_bytes() + b"\n"
    )
    factory_called = False

    def forbidden_factory(_config):
        nonlocal factory_called
        factory_called = True
        raise AssertionError("校准指纹不匹配时不应创建 Robot")

    with pytest.raises(SO100PlusConfigurationError, match="认证指纹不一致"):
        build_so100_plus_runtime(
            config,
            risk_acknowledged=True,
            robot_factory=forbidden_factory,
        )

    assert factory_called is False


def test_so100_plus_runtime_fails_closed_before_robot_when_mujoco_unavailable(
    tmp_path,
):
    config = _certified_robot_config(tmp_path)
    factory_called = False

    def forbidden_factory(_config):
        nonlocal factory_called
        factory_called = True
        raise AssertionError("MuJoCo 不可用时不应创建或连接 Robot")

    def unavailable_validator_factory():
        raise SO100PlusTrajectoryValidationUnavailableError(
            "模拟 MuJoCo 验证器不可用"
        )

    with pytest.raises(
        SO100PlusTrajectoryValidationUnavailableError,
        match="模拟 MuJoCo 验证器不可用",
    ):
        build_so100_plus_runtime(
            config,
            risk_acknowledged=True,
            robot_factory=forbidden_factory,
            trajectory_validator_factory=unavailable_validator_factory,
        )

    assert factory_called is False


def test_so100_plus_runtime_cleans_up_if_post_connect_assembly_fails(
    tmp_path,
):
    config = _certified_robot_config(tmp_path)
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
            trajectory_validator_factory=(
                _fake_trajectory_validator_factory
            ),
        )

    assert adapters[0].calls == ["connect", "stop", "disconnect"]
    assert robot.is_connected is False
