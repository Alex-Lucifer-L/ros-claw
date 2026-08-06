import math
from threading import Event, Thread

import pytest

from rosclaw_mini.arm.so100_plus import (
    SO100_PLUS_REAL_HARDWARE_PROFILE,
    SO100PlusAdapter,
    SO100PlusArmSafetyError,
    SO100PlusGripperConfig,
    SO100PlusGripperSafetyError,
    SO100PlusMotionConvergenceError,
    SO100PlusMotionConfig,
    SO100PlusMotionExecutionDisabledError,
    SO100PlusMotionPlanningDisabledError,
    SO100PlusMotionStoppedError,
    SO100PlusPIDGains,
    SO100PlusTorqueReleaseSafetyError,
)
from rosclaw_mini.arm.kinematics import JointMotionPlan
from rosclaw_mini.arm.so100_plus_trajectory_validation import (
    SO100PlusMuJoCoTrajectoryValidator,
)
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
        self.load_read_sequence = []
        self.temperature_raw = [25.0] * 7
        self.temperature_read_sequence = []
        self.torque_enabled = [1] * 7
        self.p_coefficients = [16] * 7
        self.i_coefficients = [0] * 7
        self.d_coefficients = [0] * 7
        self.locks = [0] * 7
        self.lock_write_stuck = False
        self.position_error_degrees = 0.0
        self.arm_position_error_degrees = 0.0
        self.read_calls = []
        self.write_calls = []

    def read(self, register, motor_name=None):
        self.read_calls.append((register, motor_name))
        if (
            motor_name is None
            and register == "Present_Load"
            and self.load_read_sequence
        ):
            return list(self.load_read_sequence.pop(0))
        if (
            motor_name is None
            and register == "Present_Temperature"
            and self.temperature_read_sequence
        ):
            return list(self.temperature_read_sequence.pop(0))
        if motor_name is None:
            values = {
                "Present_Position": list(self.all_positions),
                "Present_Voltage": list(self.voltage_raw),
                "Present_Current": list(self.current_raw),
                "Present_Load": list(self.load_raw),
                "Present_Temperature": list(self.temperature_raw),
                "Torque_Enable": list(self.torque_enabled),
                "P_Coefficient": list(self.p_coefficients),
                "I_Coefficient": list(self.i_coefficients),
                "D_Coefficient": list(self.d_coefficients),
                "Lock": list(self.locks),
            }
            values["Present_Position"][-1] = self.gripper_degrees
            values["Present_Load"][-1] = self.gripper_load
            return values[register]
        if register in {
            "P_Coefficient",
            "I_Coefficient",
            "D_Coefficient",
            "Lock",
        }:
            values = {
                "P_Coefficient": self.p_coefficients,
                "I_Coefficient": self.i_coefficients,
                "D_Coefficient": self.d_coefficients,
                "Lock": self.locks,
            }
            return [values[register][self.motor_names.index(motor_name)]]
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
        if register in {
            "P_Coefficient",
            "I_Coefficient",
            "D_Coefficient",
            "Lock",
        }:
            if register == "Lock" and self.lock_write_stuck:
                return
            coefficients = {
                "P_Coefficient": self.p_coefficients,
                "I_Coefficient": self.i_coefficients,
                "D_Coefficient": self.d_coefficients,
                "Lock": self.locks,
            }[register]
            indices = (
                range(len(self.motor_names))
                if motor_name is None
                else (self.motor_names.index(motor_name),)
            )
            for index in indices:
                if register == "Lock" or self.locks[index] == 0:
                    coefficients[index] = int(values)
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
        self.plan_joint_calls = []
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

    def plan_joint_pose(
        self,
        current_joint_radians,
        target_joint_radians,
        limits,
    ):
        self.plan_joint_calls.append(
            (
                tuple(current_joint_radians),
                tuple(target_joint_radians),
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
        ("Lock", 1, None),
        ("Lock", 0, "shoulder_rotation_joint"),
        ("I_Coefficient", 2, "shoulder_rotation_joint"),
        ("D_Coefficient", 32, "shoulder_rotation_joint"),
        ("Lock", 1, "shoulder_rotation_joint"),
        ("Lock", 0, "ellbow_joint"),
        ("P_Coefficient", 64, "ellbow_joint"),
        ("I_Coefficient", 2, "ellbow_joint"),
        ("D_Coefficient", 32, "ellbow_joint"),
        ("Lock", 1, "ellbow_joint"),
        ("Lock", 0, "wrist_pitch_joint"),
        ("P_Coefficient", 24, "wrist_pitch_joint"),
        ("I_Coefficient", 2, "wrist_pitch_joint"),
        ("D_Coefficient", 32, "wrist_pitch_joint"),
        ("Lock", 1, "wrist_pitch_joint"),
        ("Acceleration", 35, None),
    ]
    assert robot.bus.p_coefficients[:6] == [16, 16, 64, 24, 16, 16]
    assert robot.bus.i_coefficients[:6] == [2, 0, 2, 2, 0, 0]
    assert robot.bus.d_coefficients[:6] == [32, 0, 32, 32, 0, 0]
    assert robot.bus.locks == [1] * 7


def test_connect_twice_only_calls_robot_once():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    adapter.connect()
    first_connect_writes = list(robot.bus.write_calls)
    adapter.connect()

    assert robot.connect_calls == 1
    assert robot.bus.write_calls == first_connect_writes


def test_reconnect_restores_saved_pid_after_legacy_preset_reset():
    robot = FakeRobot()
    adapter = make_adapter(robot)
    adapter.connect()
    adapter.disconnect()

    # 模拟旧版 LeRobot 在下一次连接中写回的 SO-100 默认值。
    robot.bus.p_coefficients = [16] * 7
    robot.bus.i_coefficients = [0] * 7
    robot.bus.d_coefficients = [0] * 7
    robot.bus.locks = [0] * 7

    adapter.connect()

    assert robot.connect_calls == 2
    assert robot.bus.p_coefficients[:6] == [16, 16, 64, 24, 16, 16]
    assert robot.bus.i_coefficients[:6] == [2, 0, 2, 2, 0, 0]
    assert robot.bus.d_coefficients[:6] == [32, 0, 32, 32, 0, 0]
    assert robot.bus.locks == [1] * 7


def test_connect_fails_closed_when_eprom_lock_cannot_be_confirmed():
    robot = FakeRobot()
    robot.bus.lock_write_stuck = True
    adapter = make_adapter(robot)

    with pytest.raises(RuntimeError, match="EEPROM 写锁未能关闭"):
        adapter.connect()

    assert robot.bus.torque_enabled == [0] * 7
    assert robot.disconnect_calls == 1
    assert adapter.is_connected is False


@pytest.mark.parametrize(
    "kwargs",
    [
        {"p": -1, "i": 0, "d": 0},
        {"p": 255, "i": 0, "d": 0},
        {"p": 16, "i": True, "d": 0},
        {"p": 16, "i": 0.5, "d": 0},
    ],
)
def test_pid_gains_reject_invalid_register_values(kwargs):
    with pytest.raises(ValueError, match="0 到 254"):
        SO100PlusPIDGains(**kwargs)


def test_read_pid_gains_is_read_only_and_requires_connection():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    with pytest.raises(RuntimeError, match="读取 PID 前必须先显式连接"):
        adapter.read_pid_gains(("ellbow_joint",))

    adapter.connect()
    writes_before = list(robot.bus.write_calls)

    gains = adapter.read_pid_gains(
        ("shoulder_rotation_joint", "ellbow_joint", "wrist_pitch_joint")
    )

    assert gains == {
        "shoulder_rotation_joint": SO100PlusPIDGains(16, 2, 32),
        "ellbow_joint": SO100PlusPIDGains(64, 2, 32),
        "wrist_pitch_joint": SO100PlusPIDGains(24, 2, 32),
    }
    assert robot.bus.write_calls == writes_before


def test_set_pid_gains_requires_explicit_eprom_acknowledgement():
    robot = FakeRobot()
    adapter = make_adapter(robot)
    requested = {
        "ellbow_joint": SO100PlusPIDGains(64, 1, 16),
    }

    with pytest.raises(RuntimeError, match="设置 PID 前必须先显式连接"):
        adapter.set_pid_gains(
            requested,
            acknowledge_eprom_write=True,
        )

    adapter.connect()
    writes_before = list(robot.bus.write_calls)

    with pytest.raises(PermissionError, match="EPROM"):
        adapter.set_pid_gains(requested)

    assert robot.bus.write_calls == writes_before


def test_set_pid_gains_holds_position_unlocks_only_changed_motors_and_reads_back():
    robot = FakeRobot()
    reported = []
    adapter = make_adapter(robot, on_telemetry=reported.append)
    adapter.connect()
    robot.bus.write_calls.clear()

    previous = adapter.set_pid_gains(
        {
            "shoulder_rotation_joint": SO100PlusPIDGains(16, 0, 16),
            "ellbow_joint": SO100PlusPIDGains(64, 1, 16),
            "wrist_pitch_joint": SO100PlusPIDGains(24, 2, 32),
        },
        acknowledge_eprom_write=True,
    )

    assert previous == {
        "shoulder_rotation_joint": SO100PlusPIDGains(16, 2, 32),
        "ellbow_joint": SO100PlusPIDGains(64, 2, 32),
        "wrist_pitch_joint": SO100PlusPIDGains(24, 2, 32),
    }
    assert robot.bus.write_calls == [
        (
            "Goal_Position",
            [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, -5.0],
            None,
        ),
        ("Lock", 0, "shoulder_rotation_joint"),
        ("I_Coefficient", 0, "shoulder_rotation_joint"),
        ("D_Coefficient", 16, "shoulder_rotation_joint"),
        ("Lock", 1, "shoulder_rotation_joint"),
        ("Lock", 0, "ellbow_joint"),
        ("I_Coefficient", 1, "ellbow_joint"),
        ("D_Coefficient", 16, "ellbow_joint"),
        ("Lock", 1, "ellbow_joint"),
    ]
    assert robot.bus.i_coefficients[2] == 1
    assert robot.bus.d_coefficients[0] == 16
    assert robot.bus.d_coefficients[2] == 16
    assert robot.bus.locks == [1] * 7
    assert reported[-1].phase == "pid_updated"


def test_set_pid_gains_noop_does_not_unlock_or_rewrite_eprom():
    robot = FakeRobot()
    adapter = make_adapter(robot)
    adapter.connect()
    robot.bus.write_calls.clear()

    previous = adapter.set_pid_gains(
        {"ellbow_joint": SO100PlusPIDGains(64, 2, 32)},
        acknowledge_eprom_write=True,
    )

    assert previous == {
        "ellbow_joint": SO100PlusPIDGains(64, 2, 32),
    }
    assert robot.bus.write_calls == [
        (
            "Goal_Position",
            [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, -5.0],
            None,
        )
    ]


@pytest.mark.parametrize(
    ("motor_names", "message"),
    [
        ((), "不能为空"),
        (
            ("ellbow_joint", "ellbow_joint"),
            "不能包含重复",
        ),
        (("gripper_joint",), "只允许配置六个手臂关节"),
    ],
)
def test_pid_operations_reject_invalid_motor_lists(motor_names, message):
    robot = FakeRobot()
    adapter = make_adapter(robot)
    adapter.connect()

    with pytest.raises(ValueError, match=message):
        adapter.read_pid_gains(motor_names)


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


def test_arm_and_camera_have_independent_lifecycles_and_capture_image():
    robot = FakeRobot()
    camera = FakeCamera()
    adapter = make_adapter(robot, cameras={"right": camera})

    adapter.connect()
    assert robot.connect_calls == 1
    assert camera.connect_calls == 0
    assert adapter.cameras_connected is False

    adapter.connect_cameras()
    adapter.connect_cameras()
    images = adapter.capture_camera_images()

    assert adapter.camera_names == ("right",)
    assert adapter.cameras_connected is True
    assert camera.connect_calls == 1
    assert images == {"observation.images.right": camera.image}
    assert camera.async_read_calls == 1
    assert camera.read_calls == 0

    synchronous_images = adapter.capture_camera_images(asynchronous=False)

    assert synchronous_images == {"observation.images.right": camera.image}
    assert camera.read_calls == 1

    adapter.disconnect()

    assert robot.is_connected is False
    assert camera.is_connected is True
    assert adapter.capture_camera_images() == {
        "observation.images.right": camera.image
    }

    adapter.disconnect_cameras()
    adapter.disconnect_cameras()

    assert camera.disconnect_calls == 1
    assert camera.is_connected is False


def test_camera_connection_failure_does_not_change_connected_arm():
    robot = FakeRobot()
    first_camera = FakeCamera()
    failed_camera = FakeCamera(connect_error=OSError("camera unavailable"))
    adapter = make_adapter(
        robot,
        cameras={"right": first_camera, "phone": failed_camera},
    )
    adapter.connect()

    with pytest.raises(RuntimeError, match="phone.*机械臂状态未改变"):
        adapter.connect_cameras()

    assert robot.connect_calls == 1
    assert robot.is_connected is True
    assert first_camera.disconnect_calls == 1
    assert first_camera.is_connected is False


def test_arm_connection_failure_does_not_disconnect_camera():
    robot = FakeRobot()
    robot.bus.lock_write_stuck = True
    camera = FakeCamera()
    adapter = make_adapter(robot, cameras={"right": camera})
    adapter.connect_cameras()

    with pytest.raises(RuntimeError, match="EEPROM 写锁未能关闭"):
        adapter.connect()

    assert robot.is_connected is False
    assert camera.is_connected is True
    assert camera.disconnect_calls == 0


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
    adapter.connect_cameras()

    with pytest.raises(RuntimeError, match="图像形状.*期望"):
        adapter.capture_camera_images()

    adapter.disconnect_cameras()


def test_camera_methods_fail_clearly_when_no_camera_is_configured():
    adapter = make_adapter(FakeRobot())

    with pytest.raises(RuntimeError, match="未配置摄像头"):
        adapter.connect_cameras()
    with pytest.raises(RuntimeError, match="未配置摄像头"):
        adapter.capture_camera_images()

    adapter.disconnect_cameras()


def test_capture_requires_camera_connection_but_not_arm_connection():
    camera = FakeCamera()
    adapter = make_adapter(FakeRobot(), cameras={"right": camera})

    with pytest.raises(RuntimeError, match="right.*未连接"):
        adapter.capture_camera_images()

    adapter.connect_cameras()

    assert adapter.is_connected is False
    assert adapter.capture_camera_images() == {
        "observation.images.right": camera.image
    }


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
        ({"final_settle_seconds": -0.01}, "稳定观察时间不能为负数"),
        (
            {
                "waypoint_timeout_seconds": 0.5,
                "final_settle_seconds": 0.75,
            },
            "稳定观察时间不能大于到位超时时间",
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
        (
            {
                "stream_tracking_error_limit_degrees": 5.0,
                "stream_critical_tracking_error_limit_degrees": 5.0,
            },
            "流式紧急跟踪误差线必须大于普通记录线",
        ),
        ({"max_temperature_celsius": 0.0}, "电机温度上限必须大于 0"),
        (
            {
                "max_temperature_celsius": 60.0,
                "critical_temperature_celsius": 60.0,
            },
            "紧急温度上限必须大于普通温度上限",
        ),
        (
            {"temperature_confirmation_samples": 1},
            "连续确认次数必须是 2 到 5",
        ),
        (
            {
                "load_limit": 450.0,
                "critical_load_limit": 449.0,
            },
            "紧急负载上限不能小于普通负载上限",
        ),
        (
            {"load_confirmation_samples": 1},
            "负载连续确认次数必须是 2 到 5",
        ),
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
    assert SO100_PLUS_REAL_HARDWARE_PROFILE.wrist_pitch_p_coefficient == 24
    assert SO100_PLUS_REAL_HARDWARE_PROFILE.tuned_motor_i_coefficient == 2
    assert SO100_PLUS_REAL_HARDWARE_PROFILE.tuned_motor_d_coefficient == 32
    assert len(
        SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_driver_degrees
    ) == 6
    assert len(
        SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_tolerances_degrees
    ) == 6
    assert gripper_config.runtime_acceleration == 35
    assert gripper_config.max_step_degrees == 10.0
    assert gripper_config.settle_seconds == 2.5
    assert gripper_config.load_limit == 300.0
    assert gripper_config.position_tolerance_degrees == 3.0
    assert motion_config.final_settle_seconds == 0.75
    assert motion_config.joint_position_tolerance_degrees == 5.0
    assert motion_config.cartesian_tolerance_m == 0.012
    assert motion_config.load_limit == 930.0
    assert motion_config.critical_load_limit == 1000.0
    assert motion_config.load_confirmation_samples == 2
    assert motion_config.max_temperature_celsius == 60.0
    assert motion_config.critical_temperature_celsius == 70.0
    assert motion_config.temperature_confirmation_samples == 2
    assert motion_config.stream_frequency_hz == 30.0
    assert motion_config.stream_max_joint_speed_degrees_per_second == 12.0
    assert motion_config.stream_tracking_error_limit_degrees == 5.0
    assert motion_config.stream_critical_tracking_error_limit_degrees == 8.0


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
    robot.bus.all_positions[:6] = list(
        SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_driver_degrees
    )
    adapter = make_adapter(robot)
    adapter.connect()

    adapter.disable_torque()

    assert ("Torque_Enable", 0, None) in robot.bus.write_calls
    assert robot.bus.torque_enabled == [0] * 7
    assert robot.bus.p_coefficients[2] == 64
    assert robot.bus.p_coefficients[3] == 24
    assert robot.bus.locks == [1] * 7
    assert adapter.telemetry_history[-1].phase == "torque_disabled"


def test_disable_torque_rejects_non_rest_position_without_writing_torque():
    robot = FakeRobot()
    adapter = make_adapter(robot)
    adapter.connect()
    writes_before = list(robot.bus.write_calls)

    with pytest.raises(
        SO100PlusTorqueReleaseSafetyError,
        match="当前位置不是已验证的 follower_rest",
    ):
        adapter.disable_torque()

    assert robot.bus.write_calls == writes_before
    assert robot.bus.torque_enabled == [1] * 7


def test_emergency_disable_torque_is_explicit_and_allowed_away_from_rest():
    robot = FakeRobot()
    adapter = make_adapter(robot)
    adapter.connect()

    adapter.disable_torque(emergency=True)

    assert robot.bus.torque_enabled == [0] * 7
    assert adapter.telemetry_history[-1].phase == (
        "torque_disabled_emergency"
    )


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


def test_read_tcp_position_uses_current_joint_feedback_without_motor_write():
    class FeedbackKinematics(FakeMotionKinematics):
        def forward_position(self, joint_radians):
            return tuple(joint_radians[:3])

    robot = FakeRobot()
    kinematics = FeedbackKinematics()
    adapter = make_adapter(
        robot,
        kinematics=kinematics,
        motion_limits=make_motion_limits(),
    )
    adapter.connect()
    robot.bus.all_positions[:6] = [35.0, -1.0, 24.0, 4.0, 5.0, 6.0]
    writes_before_read = tuple(robot.bus.write_calls)

    position = adapter.read_tcp_position()

    assert position == pytest.approx((0.35, -0.01, 0.24))
    assert kinematics.convert_calls[-1] == (
        35.0,
        -1.0,
        24.0,
        4.0,
        5.0,
        6.0,
    )
    assert tuple(robot.bus.write_calls) == writes_before_read


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

    writes = goal_write_calls(robot)
    assert len(writes) == 3
    expected_arm_positions = (
        (10.25, 20.25, 30.25, 40.25, 50.25, 60.25),
        (10.75, 20.75, 30.75, 40.75, 50.75, 60.75),
        (11.0, 21.0, 31.0, 41.0, 51.0, 61.0),
    )
    for write, expected_arm in zip(
        writes,
        expected_arm_positions,
        strict=True,
    ):
        assert write[0] == "Goal_Position"
        assert write[1][:6] == pytest.approx(expected_arm)
        assert write[1][-1] == -5.0
        assert write[2] is None
    assert adapter.last_motion_plan is not kinematics.plan
    assert adapter.last_motion_plan.is_final_execution_plan is True
    assert adapter.last_motion_plan.target_joint_radians == (
        kinematics.plan.target_joint_radians
    )
    assert waits == pytest.approx(
        [1 / 30, 1 / 30, 1 / 30, 0.25, 0.25, 0.25]
    )
    assert adapter.last_settle_report is not None
    assert adapter.last_settle_report.duration_seconds == pytest.approx(0.75)
    assert len(adapter.last_settle_report.position_samples_degrees) == 4


def test_move_joints_executes_checked_joint_plan_and_preserves_gripper():
    robot = FakeRobot()
    kinematics = FakeMotionKinematics()
    limits = make_motion_limits()
    adapter = make_adapter(
        robot,
        kinematics=kinematics,
        motion_limits=limits,
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()

    adapter.move_joints((0.11, 0.21, 0.31, 0.41, 0.51, 0.61))

    assert kinematics.plan_joint_calls == [
        (
            kinematics.current_model_radians,
            (0.11, 0.21, 0.31, 0.41, 0.51, 0.61),
            limits,
        )
    ]
    writes = goal_write_calls(robot)
    assert len(writes) == len(adapter.last_motion_plan.waypoints_radians)
    assert writes[-1] == (
        "Goal_Position",
        [11.0, 21.0, 31.0, 41.0, 51.0, 61.0, -5.0],
        None,
    )


def test_execute_joint_plan_uses_prechecked_plan_without_replanning():
    robot = FakeRobot()
    kinematics = FakeMotionKinematics()
    adapter = make_adapter(
        robot,
        kinematics=kinematics,
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()
    prechecked_plan = adapter.materialize_joint_plan(kinematics.plan)

    adapter.execute_joint_plan(prechecked_plan)

    assert adapter.last_motion_plan is prechecked_plan
    assert kinematics.plan_calls == []
    assert kinematics.plan_joint_calls == []
    assert goal_write_calls(robot)[-1] == (
        "Goal_Position",
        [11.0, 21.0, 31.0, 41.0, 51.0, 61.0, -5.0],
        None,
    )


def test_execute_joint_plan_rejects_changed_actual_start_before_motor_write():
    robot = FakeRobot()
    kinematics = FakeMotionKinematics()
    adapter = make_adapter(
        robot,
        kinematics=kinematics,
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()
    final_plan = adapter.materialize_joint_plan(
        kinematics.plan,
        held_gripper_driver_degrees=-5.0,
    )
    validator = object.__new__(SO100PlusMuJoCoTrajectoryValidator)
    validator._gripper_qpos_range = (-0.2, 2.0)
    validator._sample_contacts = lambda samples, gripper_qpos: tuple(
        frozenset() for _ in samples
    )
    prechecked_plan = validator.verify_collision_free_sequence(
        (final_plan,),
        kinematics,
        gripper_qpos=math.radians(-5.0),
    ).plans[0]
    # Fake 运动学使用 driver_degrees / 100 得到模型弧度；
    # +10 对应约 5.73°，明确超过当前 5° 起点容差。
    robot.bus.all_positions[2] += 10.0

    with pytest.raises(
        SO100PlusArmSafetyError,
        match=(
            "执行前起点复核失败.*ellbow_joint.*"
            "超过现有 5.0° 关节位置容差.*未发送任何运动目标"
        ),
    ):
        adapter.execute_joint_plan(prechecked_plan)

    assert goal_write_calls(robot) == []


def test_execute_joint_plan_accepts_matching_actual_start_and_moves():
    robot = FakeRobot()
    kinematics = FakeMotionKinematics()
    adapter = make_adapter(
        robot,
        kinematics=kinematics,
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()
    prechecked_plan = adapter.materialize_joint_plan(kinematics.plan)

    adapter.execute_joint_plan(prechecked_plan)

    writes = goal_write_calls(robot)
    assert len(writes) == len(prechecked_plan.waypoints_radians)
    assert writes[-1] == (
        "Goal_Position",
        [11.0, 21.0, 31.0, 41.0, 51.0, 61.0, -5.0],
        None,
    )


def test_streaming_motor_targets_exactly_match_final_prechecked_waypoints():
    robot = FakeRobot()
    kinematics = FakeMotionKinematics()
    adapter = make_adapter(
        robot,
        kinematics=kinematics,
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(stream_frequency_hz=30.0),
    )
    adapter.connect()
    final_plan = adapter.materialize_joint_plan(
        kinematics.plan,
        held_gripper_driver_degrees=-5.0,
    )
    validator = object.__new__(SO100PlusMuJoCoTrajectoryValidator)
    validator._gripper_qpos_range = (-0.2, 2.0)
    validator._sample_contacts = lambda samples, gripper_qpos: tuple(
        frozenset() for _ in samples
    )
    verified = validator.verify_collision_free_sequence(
        (final_plan,),
        kinematics,
        gripper_qpos=math.radians(-5.0),
    )
    assert verified.sampled_joint_radians[1:] == (
        final_plan.waypoints_radians
    )

    def execution_must_not_materialize_again(*_args, **_kwargs):
        raise AssertionError("执行阶段不得再规划或插值")

    adapter.materialize_joint_plan = execution_must_not_materialize_again
    adapter.execute_joint_plan(verified.plans[0])

    writes = goal_write_calls(robot)
    assert len(writes) == len(verified.plans[0].waypoints_radians)
    assert kinematics.plan_calls == []
    assert kinematics.plan_joint_calls == []
    for write, validated_waypoint in zip(
        writes,
        verified.plans[0].waypoints_radians,
        strict=True,
    ):
        register, driver_targets, motor_name = write
        assert register == "Goal_Position"
        assert motor_name is None
        assert tuple(value / 100.0 for value in driver_targets[:6]) == (
            pytest.approx(validated_waypoint)
        )
        assert driver_targets[-1] == -5.0


def test_saved_stream_speed_materializes_slower_30hz_execution_plan():
    robot = FakeRobot()
    kinematics = FakeMotionKinematics()
    target = tuple(
        value + math.radians(4.0)
        for value in kinematics.current_model_radians
    )
    raw_plan = JointMotionPlan(
        target_position_m=kinematics.plan.target_position_m,
        current_joint_radians=kinematics.current_model_radians,
        target_joint_radians=target,
        waypoints_radians=(target,),
    )
    adapter = make_adapter(
        robot,
        kinematics=kinematics,
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )

    final_plan = adapter.materialize_joint_plan(raw_plan)

    assert final_plan.waypoint_interval_seconds == pytest.approx(1 / 30)
    assert len(final_plan.waypoints_radians) == 16
    assert final_plan.waypoints_radians[-1] == pytest.approx(target)


def test_stop_between_registration_and_first_motor_write_is_not_lost():
    robot = FakeRobot()
    adapter = make_adapter(
        robot,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(stream_frequency_hz=30.0),
    )
    adapter.connect()
    final_plan = adapter.materialize_joint_plan(
        adapter.kinematics.plan,
        held_gripper_driver_degrees=-5.0,
    )
    reached_last_prewrite_boundary = Event()
    allow_first_write = Event()
    original_write = adapter._write_motion_target

    def gated_write(*args, **kwargs):
        reached_last_prewrite_boundary.set()
        assert allow_first_write.wait(timeout=1.0)
        return original_write(*args, **kwargs)

    adapter._write_motion_target = gated_write
    adapter.begin_motion_action()
    errors = []
    worker = Thread(
        target=lambda: (
            errors.append(_capture_exception(adapter.execute_joint_plan, final_plan))
        )
    )
    worker.start()
    assert reached_last_prewrite_boundary.wait(timeout=1.0)

    adapter.stop()
    allow_first_write.set()
    worker.join(timeout=1.0)
    adapter.end_motion_action()

    assert worker.is_alive() is False
    assert len(errors) == 1
    assert isinstance(errors[0], SO100PlusMotionStoppedError)
    assert goal_write_calls(robot) == []


def _capture_exception(callable_, *args):
    try:
        callable_(*args)
    except Exception as error:
        return error
    raise AssertionError("预期调用抛出异常")


def test_stop_after_first_final_waypoint_prevents_all_later_waypoints():
    robot = FakeRobot()
    adapter = None
    wait_calls = 0

    def stop_after_first_write(_seconds):
        nonlocal wait_calls
        wait_calls += 1
        if wait_calls == 1:
            adapter.stop()
            return True
        return False

    kinematics = FakeMotionKinematics()
    target = tuple(
        value + math.radians(3.0)
        for value in kinematics.current_model_radians
    )
    raw_plan = JointMotionPlan(
        target_position_m=kinematics.plan.target_position_m,
        current_joint_radians=kinematics.current_model_radians,
        target_joint_radians=target,
        waypoints_radians=(target,),
    )
    adapter = make_adapter(
        robot,
        wait_func=stop_after_first_write,
        kinematics=kinematics,
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(stream_frequency_hz=30.0),
    )
    adapter.connect()
    final_plan = adapter.materialize_joint_plan(
        raw_plan,
        held_gripper_driver_degrees=-5.0,
    )
    assert len(final_plan.waypoints_radians) > 1

    with pytest.raises(SO100PlusMotionStoppedError, match=r"stop\(\) 取消"):
        adapter.execute_joint_plan(final_plan)

    writes = goal_write_calls(robot)
    # 第一个最终 waypoint，加 stop() 的当前位置保持。
    assert len(writes) == 2
    first_written = tuple(value / 100.0 for value in writes[0][1][:6])
    assert first_written == pytest.approx(final_plan.waypoints_radians[0])
    assert all(
        tuple(value / 100.0 for value in write[1][:6])
        != pytest.approx(final_plan.waypoints_radians[-1])
        for write in writes
    )


def test_move_to_holds_measured_position_when_joint_misses_target():
    robot = FakeRobot()
    robot.bus.arm_position_error_degrees = 5.01
    adapter = make_adapter(
        robot,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(
            waypoint_timeout_seconds=0.5,
            waypoint_poll_interval_seconds=0.25,
            final_settle_seconds=0.0,
        ),
    )
    adapter.connect()

    with pytest.raises(
        SO100PlusArmSafetyError,
        match="关节 .*_joint 在 0.5 秒内未到位：目标.*实测.*跟踪误差",
    ):
        adapter.move_to(0.3, 0.0, 0.2)

    writes = goal_write_calls(robot)
    assert writes[-2] == (
        "Goal_Position",
        [11.0, 21.0, 31.0, 41.0, 51.0, 61.0, -5.0],
        None,
    )
    assert writes[-1][1] == pytest.approx(
        [16.01, 26.01, 36.01, 46.01, 56.01, 66.01, -5.0]
    )


def test_move_to_ignores_one_load_spike_that_immediately_clears():
    robot = FakeRobot()
    adapter = make_adapter(
        robot,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()
    baseline = [100.0] * 7
    spike = list(baseline)
    spike[2] = 500.0
    robot.bus.load_raw = list(baseline)
    robot.bus.load_read_sequence = [
        baseline,
        spike,
    ]

    adapter.move_to(0.3, 0.0, 0.2)

    assert any(
        telemetry.load_magnitude[2] == 500.0
        for telemetry in adapter.telemetry_history
    )


def test_move_to_allows_sustained_load_below_new_930_limit():
    robot = FakeRobot()
    robot.bus.arm_position_error_degrees = 2.0

    def raise_load_during_wait(_seconds):
        robot.bus.load_raw[3] = 813.0
        return False

    adapter = make_adapter(
        robot,
        wait_func=raise_load_during_wait,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()

    adapter.move_to(0.3, 0.0, 0.2)

    assert any(
        telemetry.load_magnitude[3] == 813.0
        for telemetry in adapter.telemetry_history
    )


def test_move_to_holds_position_when_load_reaches_930_twice():
    robot = FakeRobot()
    robot.bus.arm_position_error_degrees = 2.0

    def raise_load_during_wait(_seconds):
        robot.bus.load_raw[3] = 930.0
        return False

    adapter = make_adapter(
        robot,
        wait_func=raise_load_during_wait,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()

    with pytest.raises(
        SO100PlusArmSafetyError,
        match=r"负载 930\.0 连续 2 次达到限制 930\.0",
    ):
        adapter.move_to(0.3, 0.0, 0.2)


def test_move_to_holds_immediately_at_critical_load_limit():
    robot = FakeRobot()

    def reach_critical_load_during_wait(_seconds):
        robot.bus.load_raw[2] = 1000.0
        return False

    adapter = make_adapter(
        robot,
        wait_func=reach_critical_load_during_wait,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()

    with pytest.raises(
        SO100PlusArmSafetyError,
        match=r"ellbow_joint 负载 1000\.0 达到紧急限制 1000\.0",
    ):
        adapter.move_to(0.3, 0.0, 0.2)

    writes = goal_write_calls(robot)
    assert writes[-1][1] == pytest.approx(
        [11.0, 21.0, 31.0, 41.0, 51.0, 61.0, -5.0]
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


def test_move_to_ignores_one_temperature_spike_that_immediately_clears():
    robot = FakeRobot()
    adapter = make_adapter(
        robot,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()
    spike = [37.0] * 7
    spike[3] = 64.0
    robot.bus.temperature_raw = [37.0] * 7
    robot.bus.temperature_read_sequence = [
        [37.0] * 7,
        spike,
    ]

    adapter.move_to(0.3, 0.0, 0.2)

    assert any(
        telemetry.temperature_raw[3] == 64.0
        for telemetry in adapter.telemetry_history
    )


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
    assert len(writes) == len(adapter.last_motion_plan.waypoints_radians) + 1
    assert writes[-1][1] == pytest.approx(
        [13.0, 23.0, 33.0, 43.0, 53.0, 63.0, -5.0]
    )


def test_move_to_holds_immediately_at_critical_temperature_limit():
    robot = FakeRobot()

    def reach_critical_temperature_during_wait(_seconds):
        robot.bus.temperature_raw[3] = 70.0
        return False

    adapter = make_adapter(
        robot,
        wait_func=reach_critical_temperature_during_wait,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()

    with pytest.raises(
        SO100PlusArmSafetyError,
        match=r"wrist_pitch_joint 温度 70\.0°C 达到紧急限制",
    ):
        adapter.move_to(0.3, 0.0, 0.2)

    writes = goal_write_calls(robot)
    assert writes[-1][1] == pytest.approx(
        [11.0, 21.0, 31.0, 41.0, 51.0, 61.0, -5.0]
    )


def test_move_to_records_cartesian_error_exceeding_twelve_mm_without_hold():
    class CartesianMissKinematics(FakeMotionKinematics):
        def forward_position(self, joint_radians):
            return (0.313, 0.0, 0.2)

    robot = FakeRobot()
    adapter = make_adapter(
        robot,
        kinematics=CartesianMissKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(),
    )
    adapter.connect()

    adapter.move_to(0.3, 0.0, 0.2)

    assert len(goal_write_calls(robot)) == len(
        adapter.last_motion_plan.waypoints_radians
    )
    assert adapter.last_cartesian_target_m == pytest.approx((0.3, 0.0, 0.2))
    assert adapter.last_cartesian_actual_m == pytest.approx((0.313, 0.0, 0.2))
    assert adapter.last_cartesian_error_m == pytest.approx(0.013)


def test_stop_cancels_arm_motion_and_holds_all_joints():
    robot = FakeRobot()
    robot.bus.arm_position_error_degrees = 5.01
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
    robot.bus.arm_position_error_degrees = 5.01
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

    assert waits == [0.25, 0.25, 0.25, 0.25]
    assert len(goal_write_calls(robot)) == 1
    phases = [item.phase for item in adapter.telemetry_history]
    assert "arm_start" in phases
    assert phases[-5:] == ["arm_step"] * 5


def test_final_settle_report_records_joint_and_tcp_jitter():
    class PositionSensitiveKinematics(FakeMotionKinematics):
        def forward_position(self, joint_radians):
            return tuple(joint_radians[:3])

    robot = FakeRobot()
    first_joint_samples = iter((10.9, 11.1, 11.0))

    def vary_position_during_settle(_seconds):
        robot.bus.all_positions[0] = next(first_joint_samples)
        return False

    adapter = make_adapter(
        robot,
        wait_func=vary_position_during_settle,
        kinematics=PositionSensitiveKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(
            stream_frequency_hz=None,
            cartesian_tolerance_m=1.0,
        ),
    )
    adapter.connect()

    adapter.move_to(0.3, 0.0, 0.2)

    report = adapter.last_settle_report
    assert report is not None
    assert report.duration_seconds == pytest.approx(0.75)
    assert tuple(
        sample[0] for sample in report.position_samples_degrees
    ) == pytest.approx((11.0, 10.9, 11.1, 11.0))
    assert report.position_span_degrees[0] == pytest.approx(0.2)
    assert report.tcp_min_m[0] == pytest.approx(0.109)
    assert report.tcp_max_m[0] == pytest.approx(0.111)
    assert report.tcp_mean_m[0] == pytest.approx(0.11)


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
    assert waits == pytest.approx([0.05] * 7 + [0.25] * 3)
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


def test_streaming_motion_continues_through_record_only_tracking_error():
    robot = FakeRobot()
    waits = []

    def clear_record_only_error_before_final_settle(seconds):
        waits.append(seconds)
        # 两个最终流式 waypoint 都允许 6° 普通滞后继续；
        # 只在最终到位轮询前模拟舵机追到目标。
        if len(waits) == 2:
            robot.bus.all_positions[:6] = [
                value - 6.0 for value in robot.bus.all_positions[:6]
            ]
            robot.bus.arm_position_error_degrees = 0.0
        return False

    adapter = make_adapter(
        robot,
        wait_func=clear_record_only_error_before_final_settle,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(
            stream_frequency_hz=20.0,
            stream_tracking_error_limit_degrees=5.0,
            stream_critical_tracking_error_limit_degrees=8.0,
        ),
    )
    adapter.connect()
    robot.bus.arm_position_error_degrees = 6.0

    adapter.move_to(0.3, 0.0, 0.2)

    assert len(goal_write_calls(robot)) == len(
        adapter.last_motion_plan.waypoints_radians
    )
    assert "arm_stream_catchup" not in (
        item.phase for item in adapter.telemetry_history
    )


def test_streaming_motion_pauses_until_critical_feedback_catches_up():
    robot = FakeRobot()
    kinematics = FakeMotionKinematics()
    waits = []

    def catch_up_after_first_pause(seconds):
        waits.append(seconds)
        # 第一次 wait 是第一个 waypoint 的流式间隔；第二次
        # 是跟踪误差超线后的只读追赶等待。
        if len(waits) == 2:
            robot.bus.all_positions[:6] = [
                value - 9.0 for value in robot.bus.all_positions[:6]
            ]
            robot.bus.arm_position_error_degrees = 0.0
        return False

    adapter = make_adapter(
        robot,
        wait_func=catch_up_after_first_pause,
        kinematics=kinematics,
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(
            stream_frequency_hz=20.0,
            stream_tracking_error_limit_degrees=5.0,
        ),
    )
    adapter.connect()
    # 起点保持匹配，只在第一条流式目标写入后注入跟踪误差。
    robot.bus.arm_position_error_degrees = 9.0

    adapter.move_to(0.3, 0.0, 0.2)

    writes = goal_write_calls(robot)
    assert len(writes) == len(adapter.last_motion_plan.waypoints_radians)
    written_waypoints = [
        tuple(value / 100.0 for value in write[1][:6])
        for write in writes
    ]
    for written, verified in zip(
        written_waypoints,
        adapter.last_motion_plan.waypoints_radians,
        strict=True,
    ):
        assert written == pytest.approx(verified)
    assert "arm_stream_catchup" in (
        item.phase for item in adapter.telemetry_history
    )


def test_streaming_motion_holds_when_feedback_never_catches_up():
    robot = FakeRobot()
    adapter = make_adapter(
        robot,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(
            waypoint_timeout_seconds=0.1,
            waypoint_poll_interval_seconds=0.05,
            final_settle_seconds=0.0,
            stream_frequency_hz=20.0,
            stream_tracking_error_limit_degrees=5.0,
        ),
    )
    adapter.connect()
    robot.bus.arm_position_error_degrees = 9.0

    with pytest.raises(
        SO100PlusMotionConvergenceError,
        match="在 0.1 秒内未追上最后一个已验证目标",
    ):
        adapter.move_to(0.3, 0.0, 0.2)

    writes = goal_write_calls(robot)
    # 只写入第一个已验证 waypoint，然后在超时时保持实测位置。
    assert len(writes) == 2
    assert tuple(value / 100.0 for value in writes[0][1][:6]) == (
        pytest.approx(adapter.last_motion_plan.waypoints_radians[0])
    )


def test_stop_during_stream_catchup_prevents_later_waypoints():
    robot = FakeRobot()
    wait_calls = 0
    adapter = None

    def stop_during_catchup(_seconds):
        nonlocal wait_calls
        wait_calls += 1
        if wait_calls == 2:
            adapter.stop()
        return False

    adapter = make_adapter(
        robot,
        wait_func=stop_during_catchup,
        kinematics=FakeMotionKinematics(),
        motion_limits=make_motion_limits(),
        motion_config=SO100PlusMotionConfig(
            stream_frequency_hz=20.0,
            stream_tracking_error_limit_degrees=5.0,
        ),
    )
    adapter.connect()
    robot.bus.arm_position_error_degrees = 9.0

    with pytest.raises(SO100PlusMotionStoppedError, match=r"stop\(\) 取消"):
        adapter.move_to(0.3, 0.0, 0.2)

    writes = goal_write_calls(robot)
    assert len(writes) == 2
    assert tuple(value / 100.0 for value in writes[0][1][:6]) == (
        pytest.approx(adapter.last_motion_plan.waypoints_radians[0])
    )


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
