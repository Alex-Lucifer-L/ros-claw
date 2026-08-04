"""在 MuJoCo UI 中循环播放 SO-100 Plus 模型 +Z 轨迹。"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

from rosclaw_mini.arm.kinematics import (
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
    SO100PlusKinematics,
)


DEFAULT_MODEL_PATH = Path(
    "lerobot-joycon_plus/lerobot/common/robot_devices/controllers/scene_plus.xml"
)
MOVE_TO_MARKER_RADIUS_M = 0.005


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="在 MuJoCo 窗口中慢速循环播放模型 +Z 动作。"
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    parser.add_argument("--delta-z-cm", type=float, default=5.0)
    parser.add_argument("--move-seconds", type=float, default=8.0)
    parser.add_argument("--hold-seconds", type=float, default=3.0)
    return parser


def _smooth_fraction(fraction: float) -> float:
    """使用余弦曲线平滑起停，避免 UI 中瞬间跳变。"""

    return 0.5 - 0.5 * math.cos(math.pi * fraction)


def _mark_move_to_point(viewer, position_m: tuple[float, float, float]) -> None:
    """用红球标出 SO100PlusKinematics 实际控制的夹爪 TCP。"""

    viewer.user_scn.ngeom = 1
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[0],
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.asarray((MOVE_TO_MARKER_RADIUS_M, 0.0, 0.0), dtype=float),
        np.asarray(position_m, dtype=float),
        np.eye(3, dtype=float).reshape(-1),
        np.asarray((1.0, 0.0, 0.0, 1.0), dtype=np.float32),
    )


def _show_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer,
    kinematics: SO100PlusKinematics,
    qpos: np.ndarray,
) -> None:
    data.qpos[:6] = qpos
    data.qpos[6] = 0.0
    mujoco.mj_forward(model, data)
    _mark_move_to_point(viewer, kinematics.forward_position(qpos))
    viewer.sync()


def _animate(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer,
    kinematics: SO100PlusKinematics,
    start: np.ndarray,
    target: np.ndarray,
    duration_seconds: float,
) -> bool:
    start_time = time.monotonic()
    while viewer.is_running():
        elapsed = time.monotonic() - start_time
        fraction = min(elapsed / duration_seconds, 1.0)
        smooth = _smooth_fraction(fraction)
        qpos = start + (target - start) * smooth
        _show_pose(model, data, viewer, kinematics, qpos)
        if fraction >= 1.0:
            return True
        time.sleep(1.0 / 60.0)
    return False


def _hold(viewer, duration_seconds: float) -> bool:
    end_time = time.monotonic() + duration_seconds
    while viewer.is_running() and time.monotonic() < end_time:
        viewer.sync()
        time.sleep(1.0 / 60.0)
    return viewer.is_running()


def main() -> int:
    args = build_parser().parse_args()
    if not math.isfinite(args.delta_z_cm) or not 0 < args.delta_z_cm <= 5.0:
        raise SystemExit("UI 预览的 delta-z-cm 必须大于 0 且不超过 5。")
    if not math.isfinite(args.move_seconds) or args.move_seconds <= 0:
        raise SystemExit("move-seconds 必须是正数。")
    if not math.isfinite(args.hold_seconds) or args.hold_seconds < 0:
        raise SystemExit("hold-seconds 不能为负数。")

    model_path = args.model.resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    rest_qpos = np.asarray(SO100_PLUS_JOYCON_INITIAL_RADIANS, dtype=float)
    kinematics = SO100PlusKinematics()
    start_position = kinematics.forward_position(rest_qpos)
    target_position = (
        start_position[0],
        start_position[1],
        start_position[2] + args.delta_z_cm / 100.0,
    )
    target_qpos = np.asarray(
        kinematics.solve_position(rest_qpos, target_position),
        dtype=float,
    )

    print(f"模型：{model_path}", flush=True)
    print(f"JoyCon 初始转换姿态 TCP XYZ(m)：{start_position}", flush=True)
    print(f"夹爪 TCP 目标 XYZ(m)：{target_position}", flush=True)
    print("红球 = move_to() 使用绝对 XYZ 控制的夹爪 TCP。", flush=True)
    print("它位于两根夹指尖端之间的夹持中心。", flush=True)
    print("窗口会循环播放上升和返回；关闭窗口即可结束。", flush=True)

    data.qpos[:6] = rest_qpos
    data.qpos[6] = 0.0
    mujoco.mj_forward(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = (0.22, 0.0, 0.16)
        viewer.cam.distance = 0.62
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -22.0
        while viewer.is_running():
            if not _animate(
                model,
                data,
                viewer,
                kinematics,
                rest_qpos,
                target_qpos,
                args.move_seconds,
            ):
                break
            if not _hold(viewer, args.hold_seconds):
                break
            if not _animate(
                model,
                data,
                viewer,
                kinematics,
                target_qpos,
                rest_qpos,
                args.move_seconds,
            ):
                break
            if not _hold(viewer, args.hold_seconds):
                break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
