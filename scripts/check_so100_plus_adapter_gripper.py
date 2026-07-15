"""通过正式 SO100PlusAdapter 手动验证一次夹爪开合。"""

from __future__ import annotations

import argparse
from pathlib import Path

from rosclaw_mini.arm.so100_plus import (
    GRIPPER_MOTOR_NAME,
    SO100PlusAdapter,
    SO100PlusGripperConfig,
    SO100PlusGripperSafetyError,
    SO100PlusTelemetry,
)
from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_robot,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="通过正式适配器分步张开和闭合 SO-100 Plus 夹爪。"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration-dir", required=True, type=Path)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument(
        "--acknowledge-production-gripper-cycle-risk",
        action="store_true",
        help="确认已准备机械支撑和断电手段，并同意夹爪完整开合",
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
    if not args.acknowledge_production_gripper_cycle_risk:
        raise SystemExit(
            "已停止：必须显式确认正式夹爪适配器的真机运动风险。"
        )

    robot_config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )
    robot = create_so100_plus_robot(robot_config)
    gripper_config = SO100PlusGripperConfig(
        follower_name=args.follower_name,
        open_degrees=60.0,
        close_degrees=-5.0,
    )
    adapter = SO100PlusAdapter(
        robot,
        gripper_config,
        on_telemetry=print_gripper_telemetry,
    )
    follower_bus = robot.follower_arms[args.follower_name]

    print("即将连接机械臂；connect 会改变扭矩并写入电机运行配置。", flush=True)
    try:
        adapter.connect()

        print("正在逐步张开夹爪到 60°。", flush=True)
        adapter.open_gripper()
        open_position = _single_value(
            follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
        )
        print(f"张开完成，实测位置 {open_position:.3f}°。", flush=True)

        print("正在逐步闭合夹爪到 -5°。", flush=True)
        adapter.close_gripper()
        close_position = _single_value(
            follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
        )
        print(f"闭合完成，实测位置 {close_position:.3f}°。", flush=True)
    except SO100PlusGripperSafetyError as exc:
        print(f"安全停止：{exc}", flush=True)
        return 2
    finally:
        if getattr(robot, "is_connected", False):
            adapter.disconnect()
        elif getattr(follower_bus, "is_connected", False):
            follower_bus.disconnect()
        print("串口已关闭；disconnect 不是急停，扭矩可能仍然启用。", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
