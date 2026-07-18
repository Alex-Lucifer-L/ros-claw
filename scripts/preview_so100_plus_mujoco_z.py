"""使用仓库现有 MuJoCo 模型离线预览 SO-100 Plus 的模型 +Z 动作。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from rosclaw_mini.arm.kinematics import (
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
    SO100PlusKinematics,
)


DEFAULT_MODEL_PATH = Path(
    "lerobot-joycon_plus/lerobot/common/robot_devices/controllers/scene_plus.xml"
)
DEFAULT_OUTPUT_PATH = Path("artifacts/so100_plus_mujoco_plus_z.png")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="离线渲染 SO-100 Plus home 姿态和模型 +Z 目标姿态。"
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    parser.add_argument("--delta-z-cm", type=float, default=5.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_PATH)
    return parser


def _render_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos: np.ndarray,
) -> tuple[np.ndarray, tuple[float, float, float]]:
    data.qpos[:6] = qpos
    data.qpos[6] = 0.0
    mujoco.mj_forward(model, data)

    camera = mujoco.MjvCamera()
    camera.lookat[:] = (0.22, 0.0, 0.16)
    camera.distance = 0.62
    camera.azimuth = 135.0
    camera.elevation = -22.0
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render().copy()
    gripper_body_position = tuple(float(value) for value in data.body("gripper").xpos)
    return pixels, gripper_body_position


def _label_image(pixels: np.ndarray, label: str) -> Image.Image:
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, image.width, 34), fill=(0, 0, 0))
    draw.text((12, 10), label, fill=(255, 255, 255))
    return image


def main() -> int:
    args = build_parser().parse_args()
    if not np.isfinite(args.delta_z_cm) or not 0 < args.delta_z_cm <= 10.0:
        raise SystemExit("delta-z-cm 必须大于 0 且不超过 10。")

    model_path = args.model.resolve()
    if not model_path.is_file():
        raise SystemExit(f"MuJoCo 模型不存在：{model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    rest_qpos = np.asarray(SO100_PLUS_JOYCON_INITIAL_RADIANS, dtype=float)
    kinematics = SO100PlusKinematics()
    tcp_start_m = kinematics.forward_position(rest_qpos)
    tcp_target_m = (
        tcp_start_m[0],
        tcp_start_m[1],
        tcp_start_m[2] + args.delta_z_cm / 100.0,
    )
    target_qpos = np.asarray(
        kinematics.solve_position(rest_qpos, tcp_target_m),
        dtype=float,
    )

    start_pixels, gripper_start_m = _render_pose(model, data, rest_qpos)
    target_pixels, gripper_target_m = _render_pose(model, data, target_qpos)
    gripper_delta_m = tuple(
        target - start
        for start, target in zip(gripper_start_m, gripper_target_m, strict=True)
    )

    start_image = _label_image(
        start_pixels,
        f"JOYCON REST | gripper body z={gripper_start_m[2]:.4f} m",
    )
    target_image = _label_image(
        target_pixels,
        f"MODEL +Z {args.delta_z_cm:.1f} cm | gripper body z={gripper_target_m[2]:.4f} m",
    )
    comparison = Image.new(
        "RGB",
        (start_image.width + target_image.width, start_image.height),
        color=(255, 255, 255),
    )
    comparison.paste(start_image, (0, 0))
    comparison.paste(target_image, (start_image.width, 0))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    comparison.save(args.output)

    summary = {
        "model": str(model_path),
        "tcp_start_m": tcp_start_m,
        "tcp_target_m": tcp_target_m,
        "gripper_body_start_m": gripper_start_m,
        "gripper_body_target_m": gripper_target_m,
        "gripper_body_delta_m": gripper_delta_m,
        "target_joint_radians": target_qpos.tolist(),
        "output": str(args.output.resolve()),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
