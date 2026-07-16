import math

import pytest

from rosclaw_mini.arm.so100_plus import (
    SO100_PLUS_REAL_HARDWARE_PROFILE,
    SO100PlusAdapter,
    SO100PlusArmSafetyError,
    SO100PlusGripperConfig,
    SO100PlusGripperSafetyError,
    SO100PlusMotionConfig,
    SO100PlusMotionExecutionDisabledError,
    SO100PlusMotionPlanningDisabledError,
    SO100PlusMotionStoppedError,
)
from rosclaw_mini.arm.kinematics import JointMotionPlan
from rosclaw_mini.safety.limits import (
    AxisLimits,
    JointLimits,
    MotionLimits,
    WorkspaceLimits,
)


class FakeFollowerBus:
    def __init__(self):
        self.motor_names = (
            "shoulder_rotation_joint",
            "shoulder_pitch_joint",
            "ellbow_joint",
            "wrist_pitch_joint",
            "wrist_jaw_joint",
            "wrist_roll_joint",
            "gripper_joint",
        )
        self.all_positions = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, -5.0]
        self.gripper_degrees = -5.0
        self.gripper_load = 0.0
        self.voltage_raw = [120.0] * 7
        self.current_raw = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
        self.load_raw = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 0.0]
        self.temperature_raw = [25.0] * 7
        self.torque_enabled = [1] * 7
        self.p_coefficients = [16] * 7
        self.position_error_degrees = 0.0
        self.arm_position_error_degrees = 0.0
        self.read_calls = []
        self.write_calls = []

    def read(self, register, motor_name=None):
        self.read_calls.append((register, motor_name))
        if motor_name is None:
            values = {
                "Present_Position": list(self.all_positions),
                "Present_Voltage": list(self.voltage_raw),
                "Present_Current": list(self.current_raw),
                "Present_Load": list(self.load_raw),
                "Present_Temperature": list(self.temperature_raw),
                "Torque_Enable": list(self.torque_enabled),
                "P_Coefficient": list(self.p_coefficients),
            }
            values["Present_Position"][-1] = self.gripper_degrees
            values["Present_Load"][-1] = self.gripper_load
            return values[register]
        if motor_name == "ellbow_joint":
            assert register == "P_Coefficient"
            return [self.p_coefficients[2]]
        assert motor_name == "gripper_joint"
        values = {
            "Present_Position": self.gripper_degrees,
            "Present_Load": self.gripper_load,
        }
        return [values[register]]

    def write(self, register, values, motor_name=None):
        self.write_calls.append((register, values, motor_name))
        if register == "Acceleration":
            return
        if register == "Torque_Enable":
            self.torque_enabled = [int(values)] * 7
            return
        if register == "P_Coefficient":
            if motor_name is None:
                self.p_coefficients = [int(values)] * 7
            else:
                assert motor_name == "ellbow_joint"
                self.p_coefficients[2] = int(values)
            return
        if motor_name == "gripper_joint":
            self.gripper_degrees = values[0] + self.position_error_degrees
        else:
            self.all_positions = list(values)
            for index in range(6):
                self.all_positions[index] += self.arm_position_error_degrees
            self.gripper_degrees = self.all_positions[-1]


class FakeRobot:
    """模拟 LeRobot 的 ManipulatorRobot，不会控制真实机械臂。"""

    def __init__(self):
        self.is_connected = False
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.bus = FakeFollowerBus()
        self.follower_arms = {"right": self.bus}

    def connect(self):
        self.connect_calls += 1
        self.is_connected = True

    def disconnect(self):
        self.disconnect_calls += 1
        self.is_connected = False


class FakeImage:
    def __init__(self, shape=(480, 640, 3)):
        self.shape = shape


class FakeCamera:
    def __init__(self, *, on_connect=None, connect_error=None, image=None):
        self.height = 480
        self.width = 640
        self.channels = 3
        self.is_connected = False
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.read_calls = 0
        self.async_read_calls = 0
        self.on_connect = on_connect
        self.connect_error = connect_error
        self.image = image or FakeImage()

    def connect(self):
        self.connect_calls += 1
        if self.on_connect is not None:
            self.on_connect()
        if self.connect_error is not None:
            raise self.connect_error
        self.is_connected = True

    def read(self):
        self.read_calls += 1
        return self.image

    def async_read(self):
        self.async_read_calls += 1
        return self.image

    def disconnect(self):
        self.disconnect_calls += 1
        self.is_connected = False


class FakeMotionKinematics:
    """记录适配器交给运动学层的输入，不执行真实 IK。"""

    def __init__(self):
        self.convert_calls = []
        self.plan_calls = []
        self.current_model_radians = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
        self.plan = JointMotionPlan(
            target_position_m=(0.3, 0.0, 0.2),
            current_joint_radians=self.current_model_radians,
            target_joint_radians=(0.11, 0.21, 0.31, 0.41, 0.51, 0.61),
            waypoints_radians=((0.11, 0.21, 0.31, 0.41, 0.51, 0.61),),
        )

    def driver_degrees_to_model_radians(self, driver_degrees):
        self.convert_calls.append(tuple(driver_degrees))
        return tuple(value / 100 for value in driver_degrees)

    def model_radians_to_driver_degrees(self, model_radians):
        return tuple(value * 100 for value in model_radians)

    def forward_position(self, joint_radians):
        return self.plan.target_position_m

    def plan_position(self, current_joint_radians, target_position_m, limits):
        self.plan_calls.append(
            (
                tuple(current_joint_radians),
                tuple(target_position_m),
                limits,
            )
        )
        return self.plan


def make_motion_limits():
    return MotionLimits(
        workspace=WorkspaceLimits(
            x=AxisLimits(0.1, 0.5),
            y=AxisLimits(-0.3, 0.3),
            z=AxisLimits(0.0, 0.4),
        ),
        joints=JointLimits(
            joint_names=(
                "shoulder_rotation_joint",
                "shoulder_pitch_joint",
                "ellbow_joint",
                "wrist_pitch_joint",
                "wrist_jaw_joint",
                "wrist_roll_joint",
            ),
            lower_radians=(-2.0,) * 6,
            upper_radians=(2.0,) * 6,
            max_step_radians=(0.1,) * 6,
        ),
    )


def make_adapter(
    robot,
    waits=None,
    wait_func=None,
    on_telemetry=None,
    kinematics=None,
    motion_limits=None,
    motion_config=None,
    cameras=None,
):
    config = SO100PlusGripperConfig(
        follower_name="right",
        open_degrees=60.0,
        close_degrees=-5.0,
    )
    return SO100PlusAdapter(
        robot,
        config,
        wait_func=(
            wait_func
            or (waits.append if waits is not None else lambda _seconds: False)
        ),
        on_telemetry=on_telemetry,
        kinematics=kinematics,
        motion_limits=motion_limits,
        motion_config=motion_config,
        cameras=cameras,
    )


def goal_write_calls(robot):
    calls = [
        call for call in robot.bus.write_calls if call[0] == "Goal_Position"
    ]
    # 正式 Adapter.connect() 的第一笔 Goal_Position 只把目标同步到
    # 当前位置，不属于被测动作。
    if robot.connect_calls and calls:
        return calls[1:]
    return calls


def test_connect():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    assert adapter.is_connected is False

    adapter.connect()

    assert adapter.is_connected is True
    assert robot.connect_calls == 1
    assert robot.bus.write_calls == [
        (
            "Goal_Position",
            [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, -5.0],
            None,
        ),
        ("P_Coefficient", 16, None),
        ("P_Coefficient", 64, "ellbow_joint"),
        ("Acceleration", 35, None),
    ]
    assert [register for register, _motor in robot.bus.read_calls] == [
        "Present_Position",
        "P_Coefficient",
        "Present_Voltage",
        "Present_Current",
        "Present_Load",
        "Present_Temperature",
    ]


def test_connect_twice_only_calls_robot_once():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    adapter.connect()
    adapter.connect()

    assert robot.connect_calls == 1
    assert robot.bus.write_calls == [
        (
            "Goal_Position",
            [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, -5.0],
            None,
        ),
        ("P_Coefficient", 16, None),
        ("P_Coefficient", 64, "ellbow_joint"),
        ("Acceleration", 35, None),
    ]


def test_disconnect():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    adapter.connect()
    adapter.disconnect()

    assert adapter.is_connected is False
    assert robot.disconnect_calls == 1


def test_disconnect_when_not_connected_does_nothing():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    adapter.disconnect()

    assert robot.disconnect_calls == 0


def test_adapter_connects_camera_before_arm_and_captures_lerobot_image_key():
    robot = FakeRobot()
    camera = FakeCamera(on_connect=lambda: assert_arm_not_connected(robot))
    adapter = make_adapter(robot, cameras={"right": camera})

    adapter.connect()
    images = adapter.capture_camera_images()

    assert adapter.camera_names == ("right",)
    assert adapter.cameras_connected is True
    assert camera.connect_calls == 1
    assert robot.connect_calls == 1
    assert images == {"observation.images.right": camera.image}
    assert camera.async_read_calls == 1
    assert camera.read_calls == 0

    synchronous_images = adapter.capture_camera_images(asynchronous=False)

    assert synchronous_images == {"observation.images.right": camera.image}
    assert camera.read_calls == 1

    adapter.disconnect()

    assert camera.disconnect_calls == 1
    assert camera.is_connected is False


def assert_arm_not_connected(robot):
    assert robot.connect_calls == 0
    assert robot.is_connected is False


def test_camera_connection_failure_keeps_arm_unconnected():
    robot = FakeRobot()
    first_camera = FakeCamera()
    failed_camera = FakeCamera(connect_error=OSError("camera unavailable"))
    adapter = make_adapter(
        robot,
        cameras={"right": first_camera, "phone": failed_camera},
    )

    with pytest.raises(RuntimeError, match="phone.*连接失败"):
        adapter.connect()

    assert robot.connect_calls == 0
    assert robot.is_connected is False
    assert first_camera.disconnect_calls == 1
    assert first_camera.is_connected is False


def test_adapter_rejects_cameras_embedded_in_lerobot_robot():
    robot = FakeRobot()
    robot.cameras = {"right": FakeCamera()}

    with pytest.raises(ValueError, match=r"SO100PlusAdapter\(cameras="):
        make_adapter(robot)

    assert robot.connect_calls == 0


def test_capture_camera_images_rejects_unexpected_shape():
    robot = FakeRobot()
    camera = FakeCamera(image=FakeImage(shape=(240, 320, 3)))
    adapter = make_adapter(robot, cameras={"right": camera})
    adapter.connect()

    with pytest.raises(RuntimeError, match="图像形状.*期望"):
        adapter.capture_camera_images()

    adapter.disconnect()


def test_open_gripper_moves_in_checked_steps_to_verified_target():
    robot = FakeRobot()
    waits = []
    adapter = make_adapter(robot, waits)
    adapter.connect()

    adapter.open_gripper()

    gripper_writes = goal_write_calls(robot)
    targets = [call[1][0] for call in gripper_writes]
    assert targets == [5.0, 15.0, 25.0, 35.0, 45.0, 55.0, 60.0]
    assert all(call[2] == "gripper_joint" for call in gripper_writes)
    assert waits == [2.5] * 7


def test_close_gripper_moves_in_checked_steps_to_verified_target():
    robot = FakeRobot()
    robot.bus.gripper_degrees = 60.0
    adapter = make_adapter(robot)
    adapter.connect()

    adapter.close_gripper()

    targets = [call[1][0] for call in goal_write_calls(robot)]
    assert targets == [50.0, 40.0, 30.0, 20.0, 10.0, 0.0, -5.0]


def test_gripper_stops_when_measured_position_is_within_tolerance():
    robot = FakeRobot()
    robot.bus.gripper_degrees = 55.0
    robot.bus.position_error_degrees = -2.0
    adapter = make_adapter(robot)
    adapter.connect()

    adapter.open_gripper()

    assert goal_write_calls(robot) == [
        ("Goal_Position", [60.0], "gripper_joint")
    ]
    assert robot.bus.gripper_degrees == 58.0


def test_gripper_command_requires_explicit_connection():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    with pytest.raises(RuntimeError, match="必须先显式连接"):
        adapter.open_gripper()

    assert robot.connect_calls == 0
    assert robot.bus.write_calls == []


def test_gripper_holds_observed_position_when_load_is_too_high():
    robot = FakeRobot()
    robot.bus.gripper_load = 301.0
    reported = []
    adapter = make_adapter(robot, on_telemetry=reported.append)
    adapter.connect()

    with pytest.raises(SO100PlusGripperSafetyError, match="负载 301.0"):
        adapter.close_gripper()

    assert goal_write_calls(robot) == [
        ("Goal_Position", [-5.0], "gripper_joint")
    ]
    assert reported[-1].phase == "gripper_start"
    assert reported[-1].load_magnitude[-1] == 301.0


def test_invalid_gripper_direction_is_rejected_during_configuration():
    with pytest.raises(ValueError, match="张开角度必须大于闭合角度"):
        SO100PlusGripperConfig(
            follower_name="right",
            open_degrees=-5.0,
            close_degrees=60.0,
        )


@pytest.mark.parametrize("runtime_acceleration", [-1, 255, 10.5, True])
def test_invalid_runtime_acceleration_is_rejected(runtime_acceleration):
    with pytest.raises(ValueError, match="0 到 254"):
        SO100PlusGripperConfig(
            follower_name="right",
            open_degrees=60.0,
            close_degrees=-5.0,
            runtime_acceleration=runtime_acceleration,
        )


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"waypoint_timeout_seconds": 0.0}, "超时时间必须大于 0"),
        ({"waypoint_poll_interval_seconds": 0.0}, "轮询间隔必须大于 0"),
        (
            {
                "waypoint_timeout_seconds": 0.5,
                "waypoint_poll_interval_seconds": 1.0,
            },
            "轮询间隔不能大于到位超时时间",
        ),
        ({"stream_frequency_hz": 4.9}, "流式轨迹频率必须在 5 到 50"),
        (
            {"stream_max_joint_speed_degrees_per_second": 60.1},
            "最大关节速度必须大于 0 且不超过 60",
        ),
        (
            {
                "joint_position_tolerance_degrees": 6.0,
                "stream_tracking_error_limit_degrees": 5.0,
            },
            "流式跟踪误差上限不能小于最终关节位置容差",
        ),
        ({"max_temperature_celsius": 0.0}, "电机温度上限必须大于 0"),
    ],
)
def test_invalid_motion_polling_config_is_rejected(kwargs, message):
    with pytest.raises(ValueError, match=message):
        SO100PlusMotionConfig(**kwargs)


def test_saved_real_hardware_profile_is_the_runtime_default():
    gripper_config = SO100PlusGripperConfig(
        follower_name="right",
        open_degrees=60.0,
        close_degrees=-5.0,
    )
    motion_config = SO100PlusMotionConfig()

    assert SO100_PLUS_REAL_HARDWARE_PROFILE.other_motor_p_coefficient == 16
    assert SO100_PLUS_REAL_HARDWARE_PROFILE.elbow_p_coefficient == 64
    assert gripper_config.runtime_acceleration == 35
    assert gripper_config.max_step_degrees == 10.0
    assert gripper_config.settle_seconds == 2.5
    assert gripper_config.position_tolerance_degrees == 3.0
    assert motion_config.joint_position_tolerance_degrees == 1.5
    assert motion_config.cartesian_tolerance_m == 0.006
    assert motion_config.load_limit == 300.0
    assert motion_config.max_temperature_celsius == 60.0
    assert motion_config.stream_frequency_hz == 20.0
    assert motion_config.stream_max_joint_speed_degrees_per_second == 20.0
    assert motion_config.stream_tracking_error_limit_degrees == 5.0


def test_telemetry_records_all_motors_and_notifies_callback():
    robot = FakeRobot()
    reported = []
    adapter = make_adapter(robot, on_telemetry=reported.append)

    adapter.connect()

    assert reported == list(adapter.telemetry_history)
    assert len(reported) == 1
    telemetry = reported[0]
    assert telemetry.phase == "connected"
    assert telemetry.motor_names == robot.bus.motor_names
    assert telemetry.voltage_raw == (120.0,) * 7
    assert telemetry.current_raw == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    assert telemetry.load_magnitude == (10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 0.0)
    assert telemetry.temperature_raw == (25.0,) * 7
    assert goal_write_calls(robot) == []


def test_stop_holds_all_joints_at_their_present_positions():
    robot = FakeRobot()
    adapter = make_adapter(robot)
    adapter.connect()

    adapter.stop()

    expected_positions = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, -5.0]
    assert ("Present_Position", None) in robot.bus.read_calls
    assert goal_write_calls(robot) == [
        ("Goal_Position", expected_positions, None)
    ]
    assert adapter.telemetry_history[-1].phase == "stopped"


def test_stop_requires_explicit_connection():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    with pytest.raises(RuntimeError, match="停止操作前必须先显式连接"):
        adapter.stop()

    assert robot.bus.read_calls == []
    assert robot.bus.write_calls == []


def test_disable_torque_maps_to_bus_and_records_telemetry():
    robot = FakeRobot()
    adapter = make_adapter(robot)
    adapter.connect()

    adapter.disable_torque()

    assert ("Torque_Enable", 0, None) in robot.bus.write_calls
    assert robot.bus.torque_enabled == [0] * 7
    assert robot.bus.p_coefficients[2] == 16
    assert adapter.telemetry_history[-1].phase == "torque_disabled"


def test_disable_torque_requires_explicit_connection():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    with pytest.raises(RuntimeError, match="关闭力矩前必须先显式连接"):
        adapter.disable_torque()

    assert robot.bus.write_calls == []


def test_stop_cancels_remaining_gripper_steps_and_holds_all_joints():
    robot = FakeRobot()
    adapter = None

    def stop_during_first_wait(_seconds):
        adapter.stop()
        return True

    adapter = make_adapter(robot, wait_func=stop_during_first_wait)
    adapter.connect()

    with pytest.raises(SO100PlusMotionStoppedError, match=r"stop\(\) 取消"):
        adapter.open_gripper()

    assert goal_write_calls(robot) == [
        ("Goal_Position", [5.0], "gripper_joint"),
        (
            "Goal_Position",
            [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 5.0],
            None,
        ),
    ]


def test_move_to_requires_explicit_motion_execution_config():
    robot = FakeRobot()
    adapter = make_adapter(
        robot,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
    )
    adapter.connect()

    with pytest.raises(
        SO100PlusMotionExecutionDisabledError,
        match="未配置经过确认的运动执行参数",
    ):
        adapter.move_to(0.3, 0.0, 0.2)

    assert goal_write_calls(robot) == []


def test_move_to_executes_planned_waypoint_and_preserves_gripper():
    robot = FakeRobot()
    waits = []
    kinematics = FakeMotionKinematics()
    adapter = make_adapter(
        robot,
        waits=waits,
        kinematics=kinematics,
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()

    adapter.move_to(0.3, 0.0, 0.2)

    assert goal_write_calls(robot) == [
        (
            "Goal_Position",
            [11.0, 21.0, 31.0, 41.0, 51.0, 61.0, -5.0],
            None,
        )
    ]
    assert adapter.last_motion_plan is kinematics.plan
    assert waits == pytest.approx([0.05])
    assert [item.phase for item in adapter.telemetry_history[-3:]] == [
        "arm_start",
        "arm_stream",
        "arm_step",
    ]


def test_move_to_holds_measured_position_when_joint_misses_target():
    robot = FakeRobot()
    robot.bus.arm_position_error_degrees = 1.51
    adapter = make_adapter(
        robot,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(
            waypoint_timeout_seconds=0.5,
            waypoint_poll_interval_seconds=0.25,
        ),
    )
    adapter.connect()

    with pytest.raises(
        SO100PlusArmSafetyError,
        match="关节 .*_joint 在 0.5 秒内未到位：目标.*实测.*跟踪误差",
    ):
        adapter.move_to(0.3, 0.0, 0.2)

    writes = goal_write_calls(robot)
    assert writes[0] == (
        "Goal_Position",
        [11.0, 21.0, 31.0, 41.0, 51.0, 61.0, -5.0],
        None,
    )
    assert writes[-1][1] == pytest.approx(
        [12.51, 22.51, 32.51, 42.51, 52.51, 62.51, -5.0]
    )


def test_move_to_holds_position_when_load_rises_over_limit():
    robot = FakeRobot()
    robot.bus.arm_position_error_degrees = 2.0

    def raise_load_during_wait(_seconds):
        robot.bus.load_raw[3] = 301.0
        return False

    adapter = make_adapter(
        robot,
        wait_func=raise_load_during_wait,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()

    with pytest.raises(SO100PlusArmSafetyError, match="负载 301.0"):
        adapter.move_to(0.3, 0.0, 0.2)

    writes = goal_write_calls(robot)
    assert len(writes) == 2
    assert writes[-1][1] == pytest.approx(
        [13.0, 23.0, 33.0, 43.0, 53.0, 63.0, -5.0]
    )


def test_move_to_allows_temperature_rise_while_below_absolute_limit():
    robot = FakeRobot()
    robot.bus.arm_position_error_degrees = 2.0
    robot.bus.temperature_raw = [37.0] * 7

    def finish_motion_at_44_celsius(_seconds):
        robot.bus.temperature_raw[6] = 44.0
        robot.bus.all_positions[:6] = [
            11.0,
            21.0,
            31.0,
            41.0,
            51.0,
            61.0,
        ]
        return False

    adapter = make_adapter(
        robot,
        wait_func=finish_motion_at_44_celsius,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()

    adapter.move_to(0.3, 0.0, 0.2)


def test_move_to_holds_position_at_absolute_temperature_limit():
    robot = FakeRobot()
    robot.bus.arm_position_error_degrees = 2.0

    def reach_temperature_limit_during_wait(_seconds):
        robot.bus.temperature_raw[4] = 60.0
        return False

    adapter = make_adapter(
        robot,
        wait_func=reach_temperature_limit_during_wait,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()

    with pytest.raises(SO100PlusArmSafetyError, match="温度 60.0°C"):
        adapter.move_to(0.3, 0.0, 0.2)

    writes = goal_write_calls(robot)
    assert len(writes) == 2
    assert writes[-1][1] == pytest.approx(
        [13.0, 23.0, 33.0, 43.0, 53.0, 63.0, -5.0]
    )


def test_move_to_holds_position_when_cartesian_error_exceeds_six_mm():
    class CartesianMissKinematics(FakeMotionKinematics):
        def forward_position(self, joint_radians):
            return (0.307, 0.0, 0.2)

    robot = FakeRobot()
    adapter = make_adapter(
        robot,
        kinematics=CartesianMissKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()

    with pytest.raises(
        SO100PlusArmSafetyError,
        match="夹爪 TCP 位置误差 7.000000 mm 超过 6.0 mm",
    ):
        adapter.move_to(0.3, 0.0, 0.2)

    assert len(goal_write_calls(robot)) == 2


def test_stop_cancels_arm_motion_and_holds_all_joints():
    robot = FakeRobot()
    robot.bus.arm_position_error_degrees = 2.0
    adapter = None

    def stop_during_wait(_seconds):
        adapter.stop()
        return True

    adapter = make_adapter(
        robot,
        wait_func=stop_during_wait,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(stream_frequency_hz=None),
    )
    adapter.connect()

    with pytest.raises(SO100PlusMotionStoppedError, match=r"stop\(\) 取消"):
        adapter.move_to(0.3, 0.0, 0.2)

    assert len(goal_write_calls(robot)) == 2
    assert adapter.telemetry_history[-1].phase == "stopped"


def test_move_to_polls_until_slow_joint_reaches_waypoint():
    robot = FakeRobot()
    robot.bus.arm_position_error_degrees = 2.0
    waits = []

    def settle_after_first_poll(seconds):
        waits.append(seconds)
        commanded_positions = goal_write_calls(robot)[0][1]
        robot.bus.all_positions = list(commanded_positions)
        robot.bus.gripper_degrees = commanded_positions[-1]
        robot.bus.arm_position_error_degrees = 0.0
        return False

    adapter = make_adapter(
        robot,
        wait_func=settle_after_first_poll,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(stream_frequency_hz=None),
    )
    adapter.connect()

    adapter.move_to(0.3, 0.0, 0.2)

    assert waits == [0.25]
    assert len(goal_write_calls(robot)) == 1
    assert [item.phase for item in adapter.telemetry_history[-3:]] == [
        "arm_start",
        "arm_step",
        "arm_step",
    ]


def test_move_to_streams_cosine_targets_without_stopping_at_each_waypoint():
    robot = FakeRobot()
    waits = []
    kinematics = FakeMotionKinematics()
    delta_radians = math.radians(4.0)
    target_joint_radians = tuple(
        value + delta_radians
        for value in kinematics.current_model_radians
    )
    kinematics.plan = JointMotionPlan(
        target_position_m=(0.3, 0.0, 0.2),
        current_joint_radians=kinematics.current_model_radians,
        target_joint_radians=target_joint_radians,
        waypoints_radians=(target_joint_radians,),
    )
    adapter = make_adapter(
        robot,
        waits=waits,
        kinematics=kinematics,
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(
            stream_frequency_hz=20.0,
            stream_max_joint_speed_degrees_per_second=20.0,
        ),
    )
    adapter.connect()

    adapter.move_to(0.3, 0.0, 0.2)

    streamed_writes = goal_write_calls(robot)
    assert len(streamed_writes) == 7
    assert waits == pytest.approx([0.05] * 7)
    assert streamed_writes[0][1][0] > 10.0
    assert streamed_writes[-1][1][:6] == pytest.approx(
        tuple(
            value * 100
            for value in kinematics.plan.target_joint_radians
        )
    )
    assert [
        item.phase
        for item in adapter.telemetry_history
        if item.phase == "arm_stream"
    ] == ["arm_stream", "arm_stream"]


def test_streaming_motion_holds_when_live_tracking_error_is_too_large():
    robot = FakeRobot()
    robot.bus.arm_position_error_degrees = 6.0
    kinematics = FakeMotionKinematics()
    adapter = make_adapter(
        robot,
        kinematics=kinematics,
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(
            stream_frequency_hz=20.0,
            stream_tracking_error_limit_degrees=5.0,
        ),
    )
    adapter.connect()

    with pytest.raises(SO100PlusArmSafetyError, match="流式轨迹关节"):
        adapter.move_to(0.3, 0.0, 0.2)

    writes = goal_write_calls(robot)
    assert len(writes) == 2


def test_motion_planning_dependencies_must_be_configured_together():
    robot = FakeRobot()

    with pytest.raises(ValueError, match="运动学和运动限制必须同时配置"):
        make_adapter(
            robot,
            kinematics=FakeMotionKinematics(),
            motion_limits=None,
        )


def test_plan_move_to_requires_explicit_connection():
    robot = FakeRobot()
    adapter = make_adapter(
        robot,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
    )

    with pytest.raises(RuntimeError, match="规划前必须先显式连接"):
        adapter.plan_move_to(0.3, 0.0, 0.2)

    assert robot.connect_calls == 0
    assert robot.bus.read_calls == []
    assert robot.bus.write_calls == []


def test_plan_move_to_stays_disabled_without_explicit_motion_limits():
    robot = FakeRobot()
    adapter = make_adapter(robot)
    adapter.connect()
    reads_after_connect = list(robot.bus.read_calls)

    with pytest.raises(
        SO100PlusMotionPlanningDisabledError,
        match="未配置运动学和经过确认的运动限制",
    ):
        adapter.plan_move_to(0.3, 0.0, 0.2)

    assert robot.bus.read_calls == reads_after_connect
    assert goal_write_calls(robot) == []


def test_plan_move_to_reads_positions_and_never_writes_goal_position():
    robot = FakeRobot()
    kinematics = FakeMotionKinematics()
    limits = make_motion_limits()
    adapter = make_adapter(
        robot,
        kinematics=kinematics,
        motion_limits=limits,
    )
    adapter.connect()

    plan = adapter.plan_move_to(0.3, 0.0, 0.2)

    assert plan is kinematics.plan
    assert adapter.last_motion_plan is plan
    assert kinematics.convert_calls == [
        (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
    ]
    assert kinematics.plan_calls == [
        (
            kinematics.current_model_radians,
            (0.3, 0.0, 0.2),
            limits,
        )
    ]
    assert ("Present_Position", None) in robot.bus.read_calls
    assert goal_write_calls(robot) == []
