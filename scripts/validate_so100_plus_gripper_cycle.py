"""手动执行 SO-100 Plus 夹爪的一次完整分步开合验证。"""

from __future__ import annotations

import argparse
from pathlib import Path

from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_robot,
)
from rosclaw_mini.arm.so100_plus_manual_check import (
    GripperCycleSafetyError,
    validate_gripper_cycle_once,
)


def print_step(step) -> None:
    print(
        f"target={step.target_degrees:6.1f}° "
        f"position={step.position_degrees:8.3f}° "
        f"load={step.load:6.1f} "
        f"current={step.current:6.1f} "
        f"temperature={step.temperature:5.1f}°C",
        flush=True,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="一次连接内分步验证 SO-100 Plus 夹爪打开和关闭。"
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration-dir", required=True, type=Path)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument(
        "--acknowledge-full-gripper-cycle-risk",
        action="store_true",
        help="确认夹爪为空载、夹口附近无人，并同意完整开合循环",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not args.acknowledge_full_gripper_cycle_risk:
        raise SystemExit(
            "已停止：必须显式添加 --acknowledge-full-gripper-cycle-risk。"
        )

    config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )
    robot = create_so100_plus_robot(config)

    print("即将连接机械臂，并执行一次分步夹爪开合循环。", flush=True)
    try:
        result = validate_gripper_cycle_once(
            robot,
            args.follower_name,
            on_step=print_step,
        )
    except GripperCycleSafetyError as exc:
        print(f"安全停止：{exc}", flush=True)
        return 2
    finally:
        print("通信关闭后扭矩可能仍然启用；disconnect 不是急停。", flush=True)

    print(
        f"夹爪循环完成：起点 {result.start_degrees:.3f}°，"
        f"共完成 {len(result.steps)} 个分步目标。",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
