"""Preview a RealSense-derived grasp plan without creating an arm Runtime."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping, Sequence
import json
import os
from pathlib import Path
import time
from typing import Any

from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.skills.arm_skills import build_arm_skills
from rosclaw_mini.vision.exceptions import VisionError
from rosclaw_mini.vision.eye_to_hand import load_eye_to_hand_calibration
from rosclaw_mini.vision.grasp_planning import (
    GraspPlanningConfig,
    build_grasp_plan,
    preview_grasp_plan,
)
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
from rosclaw_mini.workspace_scan.irregular_workspace import (
    load_default_so100_plus_irregular_workspace,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "D435i + 外参 + 正式不规则工作空间的抓取计划预览；"
            "不创建机械臂 Runtime，不执行任何动作。"
        )
    )
    parser.add_argument("--serial", required=True)
    parser.add_argument("--question", required=True)
    parser.add_argument("--eye-to-hand-calibration", type=Path, required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--timeout-ms", type=int, default=10000)
    parser.add_argument("--vlm-timeout", type=float, default=30.0)
    parser.add_argument("--vlm-model", default=None)
    parser.add_argument("--pre-grasp-height-m", type=float, default=0.08)
    parser.add_argument("--lift-height-m", type=float, default=0.08)
    parser.add_argument(
        "--approach-tcp-offset-m",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument("--maximum-cartesian-step-m", type=float, default=0.12)
    parser.add_argument("--maximum-data-age-seconds", type=float, default=30.0)
    parser.add_argument(
        "--acknowledge-camera-cloud-upload",
        action="store_true",
        help="确认当前 D435i RGB 帧会发送给配置的千问视觉服务。",
    )
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
    client_builder: Callable[..., Any] = QwenVLMClient,
    camera_builder: Callable[..., Any] = RealSenseCameraAdapter,
    service_builder: Callable[..., Any] = RealSenseLocalizationService,
    workspace_loader: Callable[..., Any] = load_default_so100_plus_irregular_workspace,
    now_ms: Callable[[], float] = lambda: time.time() * 1000.0,
    output_func: Callable[[str], None] = print,
) -> int:
    args = build_parser().parse_args(argv)
    if not args.acknowledge_camera_cloud_upload:
        output_func("已停止：需显式确认 D435i RGB 画面上传给千问。")
        return 2
    environment = os.environ if environ is None else environ
    api_key = (
        environment.get("ROSCLAW_LLM_API_KEY", "").strip()
        or environment.get("DASHSCOPE_API_KEY", "").strip()
    )
    if not api_key:
        output_func("抓取预览配置错误：缺少百炼 API Key。")
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
        localization = service.locate(args.question)
        calibration = load_eye_to_hand_calibration(
            args.eye_to_hand_calibration,
            expected_camera_serial=args.serial,
            expected_width=args.width,
            expected_height=args.height,
            require_active=True,
        )
        base_position = transform_position_estimate_to_base(
            localization.position,
            calibration,
        )
        config = GraspPlanningConfig(
            pre_grasp_height_m=args.pre_grasp_height_m,
            approach_tcp_offset_m=tuple(args.approach_tcp_offset_m),
            lift_height_m=args.lift_height_m,
            maximum_cartesian_step_m=args.maximum_cartesian_step_m,
            maximum_data_age_seconds=args.maximum_data_age_seconds,
        )
        plan = build_grasp_plan(
            base_position,
            config,
            now_timestamp_ms=now_ms(),
        )
        workspace = workspace_loader()
        skills = build_arm_skills(
            MockArmAdapter(move_duration_seconds=0.0),
            workspace_limits=workspace.endpoint_aabb,
        )
        preview = preview_grasp_plan(
            plan,
            skills,
            position_validator=workspace.validate_position,
        )
    except (ValueError, VisionError) as error:
        output_func(f"抓取计划预览失败：{error}")
        return 1
    output_func(json.dumps(preview.to_dict(), ensure_ascii=False, indent=2))
    if preview.is_safe:
        output_func("预览通过；本命令仍未创建 Runtime，也未执行运动。")
        return 0
    output_func("预览被安全门禁拒绝；未执行任何运动。")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
