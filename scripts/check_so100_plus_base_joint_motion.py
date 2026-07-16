"""让 right_follower 底座关节明显转动一次，用于隔离驱动与 IK 问题。"""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from rosclaw_mini.arm.so100_plus import (
    SO100_PLUS_REAL_HARDWARE_PROFILE,
    SO100PlusAdapter,
    SO100PlusGripperConfig,
)
from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_robot,
)
from rosclaw_mini.safety.limits import (
    choose_so100_plus_right_follower_base_test_target,
)


BASE_MOTOR_NAME = "shoulder_rotation_joint"
TEST_DELTA_DEGREES = 8.0
SETTLE_SECONDS = 3.0
OBSERVATION_SECONDS = 3.0
POSITION_TOLERANCE_DEGREES = 3.0
MIN_VISIBLE_TRAVEL_DEGREES = 4.0
LOAD_LIMIT = 300.0
MAX_TEMPERATURE_CELSIUS = 60.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="在实测安全范围内让 SO-100 Plus 底座关节转动 8 度。"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration-dir", required=True, type=Path)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument(
        "--runtime-acceleration",
        type=int,
        default=SO100_PLUS_REAL_HARDWARE_PROFILE.runtime_acceleration,
        help="本次运行时加速度，默认使用已验证值 35",
    )
    parser.add_argument(
        "--acknowledge-visible-base-motion-risk",
        action="store_true",
        help="确认左右扫动空间已清空、机械臂已托住，并同意底座明显转动 8 度",
    )
    return parser


def _single_value(values) -> float:
    if hasattr(values, "item"):
        return float(values.item())
    return float(values[0])


def _load_magnitude(value: float) -> float:
    magnitude = abs(value)
    if magnitude >= 1024:
        magnitude = abs(1024 - magnitude)
    return magnitude


def main() -> int:
    args = build_parser().parse_args()
    if not args.acknowledge_visible_base_motion_risk:
        raise SystemExit("已停止：必须显式确认底座明显转动风险。")
    if args.follower_name != "right":
        raise SystemExit("已停止：实测底座范围只适用于 follower_name=right。")

    config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )
    robot = create_so100_plus_robot(config)
    adapter = SO100PlusAdapter(
        robot,
        SO100PlusGripperConfig(
            follower_name=args.follower_name,
            open_degrees=60.0,
            close_degrees=-5.0,
            runtime_acceleration=args.runtime_acceleration,
        ),
    )
    follower_bus = robot.follower_arms[args.follower_name]
    exit_code = 0

    print("即将连接并启用扭矩；随后底座会明显转动 8 度。", flush=True)
    try:
        adapter.connect()
        start_degrees = _single_value(
            follower_bus.read("Present_Position", BASE_MOTOR_NAME)
        )
        target_degrees = choose_so100_plus_right_follower_base_test_target(
            start_degrees,
            delta_degrees=TEST_DELTA_DEGREES,
        )
        print(
            f"底座起点 {start_degrees:.3f}°，目标 {target_degrees:.3f}°。",
            flush=True,
        )
        follower_bus.write(
            "Goal_Position",
            [target_degrees],
            BASE_MOTOR_NAME,
        )
        time.sleep(SETTLE_SECONDS)

        final_degrees = _single_value(
            follower_bus.read("Present_Position", BASE_MOTOR_NAME)
        )
        final_load = _load_magnitude(
            _single_value(follower_bus.read("Present_Load", BASE_MOTOR_NAME))
        )
        final_current = _single_value(
            follower_bus.read("Present_Current", BASE_MOTOR_NAME)
        )
        final_temperature = _single_value(
            follower_bus.read("Present_Temperature", BASE_MOTOR_NAME)
        )
        travel = abs(final_degrees - start_degrees)
        tracking_error = abs(final_degrees - target_degrees)
        print(
            f"底座实测 {final_degrees:.3f}°，实际转动 {travel:.3f}°，"
            f"跟踪误差 {tracking_error:.3f}°，负载 {final_load:.1f}，"
            f"电流 {final_current:.1f}，温度 {final_temperature:.1f}。",
            flush=True,
        )
        if final_load > LOAD_LIMIT:
            raise RuntimeError(f"底座负载 {final_load:.1f} 超过 {LOAD_LIMIT:.1f}。")
        if final_temperature >= MAX_TEMPERATURE_CELSIUS:
            raise RuntimeError(
                f"底座温度 {final_temperature:.1f}°C 达到 "
                f"{MAX_TEMPERATURE_CELSIUS:.1f}°C 限制。"
            )
        if travel < MIN_VISIBLE_TRAVEL_DEGREES:
            raise RuntimeError(
                f"底座只转动 {travel:.3f}°，未达到明显运动标准。"
            )
        if tracking_error > POSITION_TOLERANCE_DEGREES:
            raise RuntimeError(
                f"底座跟踪误差 {tracking_error:.3f}° 超过 3.0°。"
            )

        print("底座明显运动诊断通过；保持 3 秒供观察。", flush=True)
        time.sleep(OBSERVATION_SECONDS)
    except Exception as error:
        print(f"安全停止：{error}", flush=True)
        exit_code = 2
    finally:
        if getattr(robot, "is_connected", False):
            try:
                adapter.stop()
                adapter.disable_torque()
                print("已保持当前位置并关闭全部力矩。", flush=True)
            except Exception as error:
                print(
                    f"力矩关闭确认失败：{error}；请立即物理断电。",
                    flush=True,
                )
                exit_code = 3
            finally:
                adapter.disconnect()
        elif getattr(follower_bus, "is_connected", False):
            follower_bus.disconnect()
        print("串口已关闭。", flush=True)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
