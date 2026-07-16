"""隔离验证 SO-100 Plus 肘关节在指定 P 增益下的负载跟踪能力。"""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_robot,
)


ELBOW_MOTOR_NAME = "ellbow_joint"
TEST_DELTA_DEGREES = -3.0
RESTORE_P_COEFFICIENT = 16
MIN_TEST_P_COEFFICIENT = 32
MAX_TEST_P_COEFFICIENT = 64
RUNTIME_ACCELERATION = 35
POLL_INTERVAL_SECONDS = 0.25
OBSERVATION_SECONDS = 6.0
POSITION_TOLERANCE_DEGREES = 1.5
LOAD_LIMIT = 300.0
MAX_TEMPERATURE_CELSIUS = 60.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "只让 SO-100 Plus 肘关节移动 -3 度，比较指定 P 增益的跟踪能力。"
        )
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration-dir", required=True, type=Path)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument(
        "--p-coefficient",
        required=True,
        type=int,
        help="本次肘关节 P 增益，只允许 32 到 64",
    )
    parser.add_argument(
        "--acknowledge-elbow-p-gain-risk",
        action="store_true",
        help="确认已托住机械臂并同意肘关节在更高 P 增益下移动 -3 度",
    )
    return parser


def _single_value(values) -> float:
    if hasattr(values, "item"):
        return float(values.item())
    return float(values[0])


def _values_tuple(values) -> tuple[float, ...]:
    return tuple(float(value) for value in values)


def _load_magnitude(value: float) -> float:
    magnitude = abs(value)
    if magnitude >= 1024:
        magnitude = abs(1024 - magnitude)
    return magnitude


def main() -> int:
    args = build_parser().parse_args()
    if not args.acknowledge_elbow_p_gain_risk:
        raise SystemExit("已停止：必须显式确认肘关节 P 增益测试风险。")
    if args.follower_name != "right":
        raise SystemExit("已停止：本次隔离测试只适用于 follower_name=right。")
    if not MIN_TEST_P_COEFFICIENT <= args.p_coefficient <= MAX_TEST_P_COEFFICIENT:
        raise SystemExit("已停止：肘关节测试 P 增益必须在 32 到 64 之间。")

    config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )
    robot = create_so100_plus_robot(config)
    follower_bus = robot.follower_arms[args.follower_name]
    exit_code = 0
    reached_target = False

    print(
        "即将启用扭矩；其他关节保持，肘关节临时使用 "
        f"P={args.p_coefficient} 移动 -3°。",
        flush=True,
    )
    try:
        robot.connect()
        motor_names = tuple(follower_bus.motor_names)
        if ELBOW_MOTOR_NAME not in motor_names:
            raise RuntimeError(f"缺少肘关节 {ELBOW_MOTOR_NAME!r}。")

        held_positions = _values_tuple(
            follower_bus.read("Present_Position")
        )
        follower_bus.write("Goal_Position", list(held_positions))
        time.sleep(0.5)

        start_degrees = _single_value(
            follower_bus.read("Present_Position", ELBOW_MOTOR_NAME)
        )
        target_degrees = start_degrees + TEST_DELTA_DEGREES

        follower_bus.write(
            "P_Coefficient",
            args.p_coefficient,
            ELBOW_MOTOR_NAME,
        )
        follower_bus.write(
            "Acceleration",
            RUNTIME_ACCELERATION,
            ELBOW_MOTOR_NAME,
        )
        actual_p = _single_value(
            follower_bus.read("P_Coefficient", ELBOW_MOTOR_NAME)
        )
        if actual_p != args.p_coefficient:
            raise RuntimeError(
                f"肘关节 P 写入失败：期望 {args.p_coefficient}，实测 {actual_p}。"
            )

        print(
            f"肘关节起点 {start_degrees:.3f}°，目标 {target_degrees:.3f}°，"
            f"P={actual_p:.0f}，Acceleration={RUNTIME_ACCELERATION}。",
            flush=True,
        )
        follower_bus.write(
            "Goal_Position",
            [target_degrees],
            ELBOW_MOTOR_NAME,
        )

        sample_count = round(
            OBSERVATION_SECONDS / POLL_INTERVAL_SECONDS
        )
        final_degrees = start_degrees
        for sample_index in range(1, sample_count + 1):
            time.sleep(POLL_INTERVAL_SECONDS)
            final_degrees = _single_value(
                follower_bus.read("Present_Position", ELBOW_MOTOR_NAME)
            )
            load = _load_magnitude(
                _single_value(
                    follower_bus.read("Present_Load", ELBOW_MOTOR_NAME)
                )
            )
            current = _single_value(
                follower_bus.read("Present_Current", ELBOW_MOTOR_NAME)
            )
            speed = _single_value(
                follower_bus.read("Present_Speed", ELBOW_MOTOR_NAME)
            )
            moving = _single_value(
                follower_bus.read("Moving", ELBOW_MOTOR_NAME)
            )
            temperature = _single_value(
                follower_bus.read("Present_Temperature", ELBOW_MOTOR_NAME)
            )
            error = abs(final_degrees - target_degrees)
            print(
                f"t={sample_index * POLL_INTERVAL_SECONDS:4.2f}s "
                f"position={final_degrees:8.3f}° error={error:6.3f}° "
                f"load={load:5.1f} current={current:5.1f} "
                f"speed={speed:5.1f} moving={moving:.0f} "
                f"temperature={temperature:.1f}",
                flush=True,
            )
            if load > LOAD_LIMIT:
                raise RuntimeError(
                    f"肘关节负载 {load:.1f} 超过 {LOAD_LIMIT:.1f}。"
                )
            if temperature >= MAX_TEMPERATURE_CELSIUS:
                raise RuntimeError(
                    f"肘关节温度 {temperature:.1f}°C 达到 "
                    f"{MAX_TEMPERATURE_CELSIUS:.1f}°C 限制。"
                )

        final_error = abs(final_degrees - target_degrees)
        travel = abs(final_degrees - start_degrees)
        reached_target = final_error <= POSITION_TOLERANCE_DEGREES
        print(
            f"结果：实际移动 {travel:.3f}°，最终误差 {final_error:.3f}°，"
            f"within_tolerance={reached_target}。",
            flush=True,
        )
        if not reached_target:
            raise RuntimeError(
                f"P={args.p_coefficient} 时肘关节误差仍为 "
                f"{final_error:.3f}°，未通过。"
            )
    except Exception as error:
        print(f"安全停止：{error}", flush=True)
        exit_code = 2
    finally:
        if getattr(follower_bus, "is_connected", False):
            try:
                present_positions = _values_tuple(
                    follower_bus.read("Present_Position")
                )
                follower_bus.write(
                    "Goal_Position",
                    list(present_positions),
                )
                follower_bus.write("Torque_Enable", 0)
                follower_bus.write(
                    "P_Coefficient",
                    RESTORE_P_COEFFICIENT,
                    ELBOW_MOTOR_NAME,
                )
                torque_enabled = tuple(
                    int(value)
                    for value in follower_bus.read("Torque_Enable")
                )
                restored_p = _single_value(
                    follower_bus.read("P_Coefficient", ELBOW_MOTOR_NAME)
                )
                if any(torque_enabled) or restored_p != RESTORE_P_COEFFICIENT:
                    raise RuntimeError(
                        f"结束状态异常：Torque_Enable={torque_enabled}，"
                        f"elbow P={restored_p}。"
                    )
                print(
                    "已保持当前位置、恢复肘关节 P=16，并关闭全部力矩。",
                    flush=True,
                )
            except Exception as error:
                print(
                    f"安全收尾失败：{error}；请立即物理断电并托住机械臂。",
                    flush=True,
                )
                exit_code = 3
            finally:
                if getattr(robot, "is_connected", False):
                    robot.disconnect()
                elif getattr(follower_bus, "is_connected", False):
                    follower_bus.disconnect()
        print("串口已关闭。", flush=True)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
