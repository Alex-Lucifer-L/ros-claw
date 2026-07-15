"""手动执行 SO-100 Plus 首次连接和只读位置检查。"""

from __future__ import annotations

import argparse
from pathlib import Path

from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_robot,
)
from rosclaw_mini.arm.so100_plus_manual_check import read_present_positions_once


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="连接一次 SO-100 Plus，只读取 7 个当前位置，然后关闭通信。"
    )
    parser.add_argument("--port", required=True, help="稳定串口别名，例如 /dev/lerobot_right")
    parser.add_argument("--calibration-dir", required=True, type=Path)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument(
        "--acknowledge-connect-risk",
        action="store_true",
        help="确认 connect 会改变扭矩并写入电机配置",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not args.acknowledge_connect_risk:
        raise SystemExit(
            "已停止：必须显式添加 --acknowledge-connect-risk，"
            "表示已经准备好机械支撑和断电手段。"
        )

    config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )
    robot = create_so100_plus_robot(config)

    print("即将连接机械臂：connect 会改变扭矩并写入电机配置。")
    try:
        positions = read_present_positions_once(robot, args.follower_name)
    finally:
        print("通信关闭后扭矩可能仍然启用；disconnect 不是急停。")

    print("读取到 7 个关节位置：")
    for motor_name, position in zip(
        robot.follower_arms[args.follower_name].motor_names,
        positions,
        strict=True,
    ):
        print(f"  {motor_name}: {position}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
