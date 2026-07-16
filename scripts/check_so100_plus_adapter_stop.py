"""通过夹爪小步运动手动验证 SO100PlusAdapter.stop()。"""

from __future__ import annotations

import argparse
from pathlib import Path
from threading import Thread
import time

from rosclaw_mini.arm.so100_plus import (
    GRIPPER_MOTOR_NAME,
    SO100_PLUS_REAL_HARDWARE_PROFILE,
    SO100PlusAdapter,
    SO100PlusGripperConfig,
    SO100PlusMotionStoppedError,
    SO100PlusTelemetry,
)
from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_robot,
)


STOP_DELAY_SECONDS = 0.5
STABILITY_WAIT_SECONDS = 1.0
MAX_STABILITY_DRIFT_DEGREES = 3.0
MIN_EXPECTED_START_DEGREES = -10.0
MAX_EXPECTED_START_DEGREES = 10.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="让夹爪小步张开，然后验证正式适配器的 stop()。"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration-dir", required=True, type=Path)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument(
        "--runtime-acceleration",
        type=int,
        default=SO100_PLUS_REAL_HARDWARE_PROFILE.runtime_acceleration,
        help="本次测试的运行时加速度，范围 0-254，默认已验证值 35",
    )
    parser.add_argument(
        "--acknowledge-stop-test-risk",
        action="store_true",
        help="确认已准备机械支撑和断电手段，并同意夹爪小步运动",
    )
    return parser


def _single_value(values) -> float:
    if hasattr(values, "item"):
        return float(values.item())
    return float(values[0])


def print_gripper_telemetry(telemetry: SO100PlusTelemetry) -> None:
    index = telemetry.motor_names.index(GRIPPER_MOTOR_NAME)
    print(
        f"telemetry phase={telemetry.phase} motor={GRIPPER_MOTOR_NAME} "
        f"voltage_raw={telemetry.voltage_raw[index]:.1f} "
        f"current_raw={telemetry.current_raw[index]:.1f} "
        f"load={telemetry.load_magnitude[index]:.1f} "
        f"temperature_raw={telemetry.temperature_raw[index]:.1f}",
        flush=True,
    )


def main() -> int:
    args = build_parser().parse_args()
    if not args.acknowledge_stop_test_risk:
        raise SystemExit("已停止：必须显式确认 stop() 真机测试风险。")

    robot_config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )
    robot = create_so100_plus_robot(robot_config)
    adapter = SO100PlusAdapter(
        robot,
        SO100PlusGripperConfig(
            follower_name=args.follower_name,
            open_degrees=60.0,
            close_degrees=-5.0,
            runtime_acceleration=args.runtime_acceleration,
        ),
        on_telemetry=print_gripper_telemetry,
    )
    follower_bus = robot.follower_arms[args.follower_name]
    worker_result = {"stopped": False, "error": None}

    def open_gripper_until_stopped() -> None:
        try:
            adapter.open_gripper()
        except SO100PlusMotionStoppedError:
            worker_result["stopped"] = True
        except BaseException as exc:  # 将子线程异常交给主线程报告。
            worker_result["error"] = exc

    print("即将连接机械臂；connect 会改变扭矩并写入电机运行配置。", flush=True)
    try:
        adapter.connect()
        start_degrees = _single_value(
            follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
        )
        print(f"夹爪起点 {start_degrees:.3f}°。", flush=True)
        if not MIN_EXPECTED_START_DEGREES <= start_degrees <= MAX_EXPECTED_START_DEGREES:
            print("安全停止：夹爪不在已验证的闭合起点范围。", flush=True)
            return 2

        worker = Thread(target=open_gripper_until_stopped, daemon=True)
        print("夹爪开始第一个张开小步。", flush=True)
        worker.start()
        time.sleep(STOP_DELAY_SECONDS)

        print("正在调用 adapter.stop() 保持当前位置。", flush=True)
        adapter.stop()
        worker.join(timeout=2.0)
        if worker.is_alive():
            print("安全停止：夹爪动作线程未及时结束。", flush=True)
            return 2
        if worker_result["error"] is not None:
            print(f"安全停止：{worker_result['error']}", flush=True)
            return 2
        if not worker_result["stopped"]:
            print("安全停止：夹爪动作没有收到取消结果。", flush=True)
            return 2

        stopped_degrees = _single_value(
            follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
        )
        time.sleep(STABILITY_WAIT_SECONDS)
        stable_degrees = _single_value(
            follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
        )
        drift_degrees = abs(stable_degrees - stopped_degrees)
        print(
            f"停止时 {stopped_degrees:.3f}°，1 秒后 {stable_degrees:.3f}°，"
            f"漂移 {drift_degrees:.3f}°。",
            flush=True,
        )
        if drift_degrees > MAX_STABILITY_DRIFT_DEGREES:
            print("安全停止：夹爪在 stop() 后仍有过大位移。", flush=True)
            return 2
    finally:
        if getattr(robot, "is_connected", False):
            adapter.disconnect()
        elif getattr(follower_bus, "is_connected", False):
            follower_bus.disconnect()
        print("串口已关闭；disconnect 不是急停，扭矩可能仍然启用。", flush=True)

    print("stop() 真机验证通过。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
