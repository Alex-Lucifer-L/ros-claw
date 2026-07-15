"""手动执行 SO-100 Plus 夹爪向打开方向最多 5 度的微动测试。"""

from __future__ import annotations

import argparse
from pathlib import Path

from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_robot,
)
from rosclaw_mini.arm.so100_plus_manual_check import nudge_gripper_open_once


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="只让 SO-100 Plus 的 gripper_joint 向打开方向微动最多 5 度。"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration-dir", required=True, type=Path)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument("--delta-degrees", required=True, type=float)
    parser.add_argument(
        "--acknowledge-gripper-motion-risk",
        action="store_true",
        help="确认夹爪为空载、夹口附近无人，并同意真实夹爪运动",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not args.acknowledge_gripper_motion_risk:
        raise SystemExit(
            "已停止：必须显式添加 --acknowledge-gripper-motion-risk。"
        )

    config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )
    robot = create_so100_plus_robot(config)

    print("即将连接机械臂，并只让 gripper_joint 向打开方向微动。")
    try:
        result = nudge_gripper_open_once(
            robot,
            args.follower_name,
            args.delta_degrees,
        )
    finally:
        print("通信关闭后扭矩可能仍然启用；disconnect 不是急停。")

    print(f"夹爪起点：{result.start_degrees:.4f}°")
    print(f"夹爪目标：{result.target_degrees:.4f}°")
    print(f"夹爪实测：{result.final_degrees:.4f}°")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
