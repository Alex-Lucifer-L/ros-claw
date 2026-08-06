"""Read-only natural-language target localization in the RealSense frame."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping, Sequence
import json
import os
from pathlib import Path
from typing import Any

from rosclaw_mini.vision.exceptions import VisionError
from rosclaw_mini.vision.eye_to_hand import load_eye_to_hand_calibration
from rosclaw_mini.vision.localization import (
    RealSenseLocalizationService,
    transform_position_estimate_to_base,
)
from rosclaw_mini.vision.realsense import RealSenseCameraAdapter
from rosclaw_mini.vision.vlm_client import (
    DEFAULT_DASHSCOPE_BASE_URL,
    DEFAULT_QWEN_VL_MODEL,
    QwenVLMClient,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "使用 RealSense 同步 RGBD 与千问 VLM 只读定位一个目标；"
            "只输出相机光学坐标，不创建机械臂 Runtime。"
        )
    )
    parser.add_argument("--serial", required=True)
    parser.add_argument("--question", required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--vlm-timeout", type=float, default=30.0)
    parser.add_argument("--vlm-model", default=None)
    parser.add_argument(
        "--eye-to-hand-calibration",
        type=Path,
        default=None,
        help=(
            "可选：已激活且与当前序列号/分辨率绑定的固定相机外参；"
            "提供后额外输出机械臂基座坐标。"
        ),
    )
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
    client_builder: Callable[..., Any] = QwenVLMClient,
    camera_builder: Callable[..., Any] = RealSenseCameraAdapter,
    service_builder: Callable[..., Any] = RealSenseLocalizationService,
    output_func: Callable[[str], None] = print,
) -> int:
    args = build_parser().parse_args(argv)
    environment = os.environ if environ is None else environ
    api_key = (
        environment.get("ROSCLAW_LLM_API_KEY", "").strip()
        or environment.get("DASHSCOPE_API_KEY", "").strip()
    )
    if not api_key:
        output_func(
            "RealSense 定位配置错误：请设置 ROSCLAW_LLM_API_KEY "
            "或 DASHSCOPE_API_KEY。"
        )
        return 2
    model = (
        args.vlm_model.strip()
        if isinstance(args.vlm_model, str) and args.vlm_model.strip()
        else environment.get("DASHSCOPE_VL_MODEL", "").strip()
        or DEFAULT_QWEN_VL_MODEL
    )
    base_url = (
        environment.get("ROSCLAW_LLM_BASE_URL", "").strip()
        or DEFAULT_DASHSCOPE_BASE_URL
    )
    try:
        client = client_builder(
            base_url=base_url,
            model=model,
            api_key=api_key,
            timeout_seconds=args.vlm_timeout,
        )
        service = service_builder(
            client=client,
            camera_factory=lambda: camera_builder(
                args.serial,
                width=args.width,
                height=args.height,
                fps=args.fps,
                timeout_ms=args.timeout_ms,
            ),
        )
        result = service.locate(args.question)
        base_position = None
        if args.eye_to_hand_calibration is not None:
            calibration = load_eye_to_hand_calibration(
                args.eye_to_hand_calibration,
                expected_camera_serial=args.serial,
                expected_width=args.width,
                expected_height=args.height,
                require_active=True,
            )
            base_position = transform_position_estimate_to_base(
                result.position,
                calibration,
            )
    except (ValueError, VisionError) as error:
        output_func(f"RealSense 目标定位失败：{error}")
        return 1
    payload = {
        "observation": result.observation.to_dict(),
        "position_estimate": result.position.to_dict(),
    }
    if base_position is not None:
        payload["base_position_estimate"] = base_position.to_dict()
    output_func(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
