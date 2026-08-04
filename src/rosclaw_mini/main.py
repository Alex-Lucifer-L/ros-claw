"""RosClaw Mini 的 JSON 命令行入口。"""

import argparse
from collections.abc import Callable, Mapping, Sequence
import json
import os
from pathlib import Path
import uuid
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.gateway.command.gateway import preflight_command
from rosclaw_mini.llm.client import LLMClient, LLMClientError
from rosclaw_mini.llm.command_generator import CommandGenerator
from rosclaw_mini.llm.intent_validator import (
    CommandIntentValidationError,
    validate_command_intent,
)
from rosclaw_mini.llm.openai_compatible_client import OpenAICompatibleClient
from rosclaw_mini.rag.context import (
    DEFAULT_RAG_MAX_CONTEXT_CHARS,
    DEFAULT_RAG_TOP_K,
    RagContextProvider,
)
from rosclaw_mini.arm.so100_plus_factory import SO100PlusRobotConfig
from rosclaw_mini.llm.command_parser import parse_json_command
from rosclaw_mini.runtime import (
    ArmRuntime,
    DEFAULT_SO100_PLUS_CALIBRATION_DIR,
    DEFAULT_SO100_PLUS_FOLLOWER_NAME,
    DEFAULT_SO100_PLUS_PORT,
    build_mock_runtime,
    build_so100_plus_runtime,
)
from rosclaw_mini.vision.exceptions import VisionError
from rosclaw_mini.vision.output import (
    format_observation_json,
    format_observation_text,
)
from rosclaw_mini.vision.service import VisionService
from rosclaw_mini.vision.vlm_client import (
    DEFAULT_DASHSCOPE_BASE_URL,
    DEFAULT_QWEN_VL_MODEL,
    QwenVLMClient,
    VLMClient,
)


InputFunction = Callable[[str], str]###定义了一个类型别名 InputFunction，它表示一个可调用对象（函数或方法），该对象接受一个字符串参数并返回一个字符串。这个类型别名用于表示输入函数的签名，通常用于从用户获取输入。
OutputFunction = Callable[[str], None]###定义了一个类型别名 OutputFunction，它表示一个可调用对象（函数或方法），该对象接受一个字符串参数并返回 None。这个类型别名用于表示输出函数的签名，通常用于向用户显示输出信息。
RuntimeBuilder = Callable[[argparse.Namespace], ArmRuntime]###定义了一个类型别名 RuntimeBuilder，它表示一个可调用对象（函数或方法），该对象接受一个 argparse.Namespace 对象作为参数并返回一个 ArmRuntime 对象。这个类型别名用于表示运行时构建器的签名，通常用于根据命令行参数创建和配置 ArmRuntime 实例。
LLMClientBuilder = Callable[..., LLMClient]
RagContextProviderBuilder = Callable[..., RagContextProvider]
VLMClientBuilder = Callable[..., VLMClient]
VisionServiceBuilder = Callable[..., VisionService]

DEFAULT_KNOWLEDGE_DIRECTORY = Path(__file__).resolve().parents[2] / "knowledge"
REAL_HARDWARE_MOTION_SKILLS = frozenset(
    {
        "move_arm",
        "move_relative",
        "unfold_arm",
        "fold_arm",
        "open_gripper",
        "close_gripper",
    }
)
EMERGENCY_EXIT_INPUTS = frozenset({"emergency_exit", "紧急退出"})


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="运行 RosClaw Mini 输入 → Gateway → Skill 命令链路。",
    )
    parser.add_argument(
        "--input-mode",
        choices=("json", "llm", "vision"),
        default="json",
        help=(
            "输入模式；默认 json，llm 使用 OpenAI-compatible 文本服务，"
            "vision 只执行结构化场景观察。"
        ),
    )
    parser.add_argument(
        "--knowledge-dir",
        type=Path,
        default=DEFAULT_KNOWLEDGE_DIRECTORY,
        help="RAG 项目知识目录；只在 llm 模式使用。",
    )
    parser.add_argument(
        "--rag-top-k",
        type=int,
        default=DEFAULT_RAG_TOP_K,
        help="每条自然语言命令最多检索的项目知识块数量。",
    )
    parser.add_argument(
        "--rag-max-context-chars",
        type=int,
        default=DEFAULT_RAG_MAX_CONTEXT_CHARS,
        help="加入 LLM Prompt 的项目知识最大字符数。",
    )
    parser.add_argument(
        "--disable-rag",
        action="store_true",
        help="LLM 模式临时只使用基础 Prompt，不加载项目知识。",
    )
    parser.add_argument(
        "--backend",
        choices=("mock", "so100_plus"),
        default="mock",
        help="机械臂后端；默认 mock，真机必须显式选择 so100_plus。",
    )
    parser.add_argument(
        "--port",
        type=Path,
        default=DEFAULT_SO100_PLUS_PORT,
        help="SO-100 Plus 串口，仅真机模式使用。",
    )
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        default=DEFAULT_SO100_PLUS_CALIBRATION_DIR,
        help="SO-100 Plus 校准目录，仅真机模式使用。",
    )
    parser.add_argument(
        "--follower-name",
        default=DEFAULT_SO100_PLUS_FOLLOWER_NAME,
        help="已登记工作空间的 follower 名称；当前只允许 right。",
    )
    parser.add_argument(
        "--acknowledge-so100-plus-risk",
        action="store_true",
        help="确认真机连接会启用力矩，后续 JSON Skill 可能产生真实运动。",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="vision 摄像头编号；默认 0。",
    )
    parser.add_argument(
        "--camera-device",
        type=Path,
        default=None,
        help=(
            "vision 摄像头绝对设备路径；优先于 --camera-index，"
            "也可用 ROSCLAW_VISION_CAMERA_DEVICE。"
        ),
    )
    parser.add_argument(
        "--vlm-model",
        default=None,
        help="vision 使用的千问视觉模型；优先于 DASHSCOPE_VL_MODEL。",
    )
    parser.add_argument(
        "--vision-question",
        default=None,
        help="提供后执行一次视觉观察并退出；省略则进入 observe/ask 交互。",
    )
    parser.add_argument(
        "--vision-image",
        type=Path,
        default=None,
        help="使用本地图像代替摄像头。",
    )
    parser.add_argument(
        "--vision-timeout",
        type=float,
        default=30.0,
        help="视觉模型请求超时秒数；默认 30。",
    )
    parser.add_argument(
        "--vision-max-width",
        type=int,
        default=1280,
        help="上传图像最大宽度，等比例缩放；默认 1280。",
    )
    parser.add_argument(
        "--vision-save-frame",
        type=Path,
        default=None,
        help="显式保存摄像头捕获帧；本地图像模式不使用。",
    )
    parser.add_argument(
        "--vision-output-format",
        choices=("text", "json"),
        default="text",
        help="视觉结果终端格式；默认 text。",
    )
    return parser


def build_runtime_from_args(args: argparse.Namespace) -> ArmRuntime:
    """根据启动参数选择后端；具体装配由 runtime 模块负责。"""

    if args.backend == "mock":
        return build_mock_runtime()

    return build_so100_plus_runtime(
        SO100PlusRobotConfig(
            port=args.port,
            calibration_dir=args.calibration_dir,
            follower_name=args.follower_name,
        ),
        risk_acknowledged=args.acknowledge_so100_plus_risk,
    )


def build_llm_client_from_environment(
    *,
    environ: Mapping[str, str],
    client_builder: LLMClientBuilder = OpenAICompatibleClient,
) -> LLMClient:
    """读取显式环境配置；本函数不发起网络请求。"""

    base_url = environ.get("ROSCLAW_LLM_BASE_URL", "").strip()
    model = environ.get("ROSCLAW_LLM_MODEL", "").strip()
    missing = tuple(
        name
        for name, value in (
            ("ROSCLAW_LLM_BASE_URL", base_url),
            ("ROSCLAW_LLM_MODEL", model),
        )
        if not value
    )
    if missing:
        raise ValueError(
            "LLM 模式缺少必填环境变量: " + ", ".join(missing)
        )

    api_key_value = environ.get("ROSCLAW_LLM_API_KEY")
    api_key = (
        api_key_value.strip()
        if api_key_value is not None and api_key_value.strip()
        else None
    )
    return client_builder(
        base_url=base_url,
        model=model,
        api_key=api_key,
    )


def build_rag_context_provider(
    *,
    knowledge_directory: Path,
    top_k: int,
) -> RagContextProvider:
    """一次加载项目知识目录；不会访问网络或机械臂。"""

    return RagContextProvider.from_directory(
        knowledge_directory,
        top_k=top_k,
    )


def build_vision_service_from_args(
    *,
    args: argparse.Namespace,
    environ: Mapping[str, str],
    client_builder: VLMClientBuilder = QwenVLMClient,
    service_builder: VisionServiceBuilder = VisionService,
) -> VisionService:
    """Build the read-only vision chain without constructing an ArmRuntime."""

    api_key = (
        environ.get("ROSCLAW_LLM_API_KEY", "").strip()
        or environ.get("DASHSCOPE_API_KEY", "").strip()
    )
    if not api_key:
        raise ValueError(
            "vision 模式缺少 API Key：请设置 ROSCLAW_LLM_API_KEY "
            "或 DASHSCOPE_API_KEY。"
        )
    model = (
        args.vlm_model.strip()
        if isinstance(args.vlm_model, str) and args.vlm_model.strip()
        else environ.get("DASHSCOPE_VL_MODEL", "").strip()
        or DEFAULT_QWEN_VL_MODEL
    )
    base_url = (
        environ.get("ROSCLAW_LLM_BASE_URL", "").strip()
        or DEFAULT_DASHSCOPE_BASE_URL
    )
    client = client_builder(
        base_url=base_url,
        model=model,
        api_key=api_key,
        timeout_seconds=args.vision_timeout,
    )
    camera_device = args.camera_device
    if camera_device is None:
        environment_device = environ.get(
            "ROSCLAW_VISION_CAMERA_DEVICE", ""
        ).strip()
        camera_device = Path(environment_device) if environment_device else None
    return service_builder(
        client=client,
        camera_index=args.camera_index,
        camera_device=camera_device,
        max_width=args.vision_max_width,
    )


def _output_vision_observation(
    observation,
    *,
    output_format: str,
    output_func: OutputFunction,
) -> None:
    if output_format == "json":
        output_func(format_observation_json(observation))
    else:
        output_func(
            "视觉观察完成：\n" + format_observation_text(observation)
        )


def run_vision_command_loop(
    service: VisionService,
    *,
    image_path: Path | None = None,
    question: str | None = None,
    save_frame_path: Path | None = None,
    output_format: str = "text",
    input_func: InputFunction = input,
    output_func: OutputFunction = print,
) -> int:
    """Run one-shot or interactive read-only visual observation."""

    def observe_once(visual_question: str | None) -> bool:
        try:
            observation = service.observe(
                question=visual_question,
                image_path=image_path,
                save_frame_path=save_frame_path,
            )
        except VisionError as error:
            output_func(f"视觉观察失败：{error}")
            return False
        _output_vision_observation(
            observation,
            output_format=output_format,
            output_func=output_func,
        )
        return True

    if question is not None:
        return 0 if observe_once(question) else 1

    while True:
        try:
            user_input = input_func(
                "视觉指令（observe / ask <问题> / exit）："
            ).strip()
        except EOFError:
            output_func("输入已结束，退出视觉模式。")
            return 0
        if user_input == "exit":
            output_func("退出视觉模式。")
            return 0
        if user_input == "observe":
            observe_once(None)
            continue
        if user_input.startswith("ask ") and user_input[4:].strip():
            observe_once(user_input[4:].strip())
            continue
        output_func("无法识别视觉指令；请输入 observe、ask <问题> 或 exit。")


def _runtime_state_for_prompt(runtime: ArmRuntime, backend: str) -> str:
    session_state = getattr(runtime, "session_state", None)
    if session_state is None:
        return f"backend={backend}; session_state=NOT_APPLICABLE"
    state_value = getattr(session_state, "value", str(session_state))
    return f"backend={backend}; session_state={state_value}"


def _shutdown_runtime(runtime: ArmRuntime, *, emergency: bool) -> None:
    if emergency:
        emergency_shutdown = getattr(runtime, "emergency_shutdown", None)
        if callable(emergency_shutdown):
            emergency_shutdown()
            return
    runtime.shutdown()

def dispatch_command(
    runtime: ArmRuntime,
    command: Command,
) -> str:
    """把已经生成的 Command 提交给现有执行链。"""

    if command.skill_name == "stop":
        stop_result = runtime.controller.request_stop(command)
        return f"停止命令执行结果: {stop_result}"

    accepted = runtime.controller.submit(command)

    if accepted:
        return (
            f"命令 {command.command_id} 已提交，"
            "正在后台执行。"
        )

    active_command_id = getattr(
        runtime.controller,
        "active_command_id",
        None,
    )
    command_detail = (
        f"（command_id={active_command_id}）"
        if active_command_id is not None
        else ""
    )
    return (
        f"当前有命令正在执行{command_detail}，本命令未提交；"
        "请等待其完成，或使用 result/stop。"
    )


def _running_command_message(runtime: ArmRuntime) -> str:
    active_command_id = getattr(
        runtime.controller,
        "active_command_id",
        None,
    )
    if active_command_id is None:
        return "当前命令仍在执行；只允许 result 或 stop。"
    return (
        f"当前命令仍在执行：command_id={active_command_id}；"
        "只允许 result 或 stop。"
    )


def _format_command_for_confirmation(command: Command) -> tuple[str, ...]:
    lines = ["解析结果：", f"skill_name: {command.skill_name}"]
    for name, value in command.params.items():
        suffix = " m" if name in {"x", "y", "z", "dx", "dy", "dz"} else ""
        lines.append(f"{name}: {value}{suffix}")
    return tuple(lines)


def _prepare_and_dispatch_command(
    runtime: ArmRuntime,
    command: Command,
    *,
    input_func: InputFunction,
    output_func: OutputFunction,
    require_motion_confirmation: bool,
    original_user_input: str | None = None,
) -> str:
    """完成入口预检、LLM 语义复核和可选真机确认后再提交。"""

    if command.skill_name == "stop":
        return dispatch_command(runtime, command)

    if runtime.controller.is_running():
        return _running_command_message(runtime)

    preflight_failure = preflight_command(command, runtime.skills)
    if preflight_failure is not None:
        return f"命令未提交：{preflight_failure.message}"

    if original_user_input is not None:
        try:
            validate_command_intent(original_user_input, command)
        except CommandIntentValidationError as error:
            return f"命令未提交：{error}"

    if (
        require_motion_confirmation
        and command.skill_name in REAL_HARDWARE_MOTION_SKILLS
    ):
        for line in _format_command_for_confirmation(command):
            output_func(line)
        try:
            answer = input_func("确认执行？[y/N]").strip().lower()
        except EOFError:
            answer = ""
        if answer not in {"y", "yes"}:
            return f"命令 {command.command_id} 已取消，未提交执行。"

    return dispatch_command(runtime, command)


def _normal_exit_allowed(runtime: ArmRuntime) -> tuple[bool, str]:
    controller = getattr(runtime, "controller", None)
    if controller is not None and controller.is_running():
        return (
            False,
            "普通 exit 已拒绝：后台命令仍在执行。"
            + _running_command_message(runtime),
        )
    state = getattr(runtime, "session_state", None)
    if state is None or state.value == "REST":
        return True, "准备退出程序。"
    if state.value == "WORK":
        suggestion = "请先执行 fold_arm，并用 result 确认已进入 REST。"
    elif state.value == "TRANSITION":
        suggestion = "请先使用 stop，等待结果后重新认证当前状态。"
    else:
        suggestion = "请先使用 revalidate_state 重新认证；确认 WORK 后可显式 fold_arm。"
    return (
        False,
        f"普通 exit 已拒绝：当前 SO-100 Plus 会话状态为 {state.value}，"
        f"尚未证明机械臂安全回到 REST。{suggestion}如需立即退出，"
        "请输入 emergency_exit（或“紧急退出”）。",
    )


def _handle_exit_input(
    runtime: ArmRuntime,
    user_input: str,
    output_func: OutputFunction,
) -> tuple[bool, bool]:
    """返回 ``(是否退出循环, 是否为紧急退出)``。"""

    if user_input in EMERGENCY_EXIT_INPUTS:
        output_func(
            "紧急退出：将立即请求 stop，等待后台动作结束后关闭力矩并"
            "断开连接；机械臂可能没有回到认证 REST，请人工支撑机械臂。"
        )
        return True, True
    if user_input != "exit":
        return False, False
    allowed, message = _normal_exit_allowed(runtime)
    output_func(message)
    return allowed, False

def run_json_command_loop(
    runtime: ArmRuntime,
    *,
    input_func: InputFunction = input,
    output_func: OutputFunction = print,
    require_motion_confirmation: bool = False,
) -> bool:
    """读取 JSON 命令，并交给现有 Gateway 与 ExecutionController。"""

    while True:
        try:
            command_text = input_func("请输入指令：")
        except EOFError:
            output_func("输入已结束，准备退出程序。")
            return False

        command_text = command_text.strip()
        should_exit, emergency = _handle_exit_input(
            runtime,
            command_text,
            output_func,
        )
        if should_exit:
            return emergency

        if command_text == "result":
            if runtime.controller.is_running():
                output_func(_running_command_message(runtime))
            else:
                result = runtime.controller.last_result()
                if result is None:
                    output_func("没有上一次命令的执行结果。")
                else:
                    output_func(f"上一次命令执行结果: {result}")
            continue

        command_id = str(uuid.uuid4())
        try:
            command = parse_json_command(command_text, command_id)
        except json.JSONDecodeError:
            output_func("输入内容不是合法 JSON")
            continue
        except ValueError:
            output_func("JSON 合法，但 Command 数据结构不合法")
            continue

        if require_motion_confirmation:
            output_func(
                _prepare_and_dispatch_command(
                    runtime,
                    command,
                    input_func=input_func,
                    output_func=output_func,
                    require_motion_confirmation=True,
                )
            )
        else:
            output_func(dispatch_command(runtime, command))

def run_llm_command_loop(
    runtime: ArmRuntime,
    generator: CommandGenerator,
    *,
    input_func: InputFunction = input,
    output_func: OutputFunction = print,
    require_motion_confirmation: bool = False,
) -> bool:
    """读取自然语言，通过 LLM 生成 Command 后进入现有执行链。"""

    while True:
        try:
            user_input = input_func("请输入自然语言指令：")
        except EOFError:
            output_func("输入已结束，准备退出程序。")
            return False

        user_input = user_input.strip()

        should_exit, emergency = _handle_exit_input(
            runtime,
            user_input,
            output_func,
        )
        if should_exit:
            return emergency

        if user_input == "result":
            if runtime.controller.is_running():
                output_func(_running_command_message(runtime))
            else:
                result = runtime.controller.last_result()

                if result is None:
                    output_func("没有上一次命令的执行结果。")
                else:
                    output_func(f"上一次命令执行结果: {result}")

            continue

        try:
            command = generator.generate(user_input)
        except LLMClientError as error:
            output_func(f"LLM 调用失败: {error}")
            continue
        except json.JSONDecodeError:
            output_func("模型返回的内容不是合法 JSON")
            continue
        except ValueError as error:
            output_func(f"模型生成的 Command 不合法: {error}")
            continue

        output_func(
            _prepare_and_dispatch_command(
                runtime,
                command,
                input_func=input_func,
                output_func=output_func,
                require_motion_confirmation=require_motion_confirmation,
                original_user_input=user_input,
            )
        )


def main(
    argv: Sequence[str] | None = None,
    *,
    input_func: InputFunction = input,
    output_func: OutputFunction = print,
    runtime_builder: RuntimeBuilder = build_runtime_from_args,
    llm_client_builder: LLMClientBuilder = OpenAICompatibleClient,
    rag_context_provider_builder: RagContextProviderBuilder = (
        build_rag_context_provider
    ),
    vlm_client_builder: VLMClientBuilder = QwenVLMClient,
    vision_service_builder: VisionServiceBuilder = VisionService,
    environ: Mapping[str, str] | None = None,
) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.rag_top_k <= 0:
        parser.error("--rag-top-k 必须大于 0")
    if args.rag_max_context_chars < 128:
        parser.error("--rag-max-context-chars 不能小于 128")

    environment = os.environ if environ is None else environ
    if args.input_mode == "vision":
        if args.camera_index < 0:
            parser.error("--camera-index 不能为负数")
        if args.camera_device is not None and not args.camera_device.is_absolute():
            parser.error("--camera-device 必须是绝对设备路径")
        if args.vision_timeout <= 0:
            parser.error("--vision-timeout 必须大于 0")
        if args.vision_max_width <= 0:
            parser.error("--vision-max-width 必须大于 0")
        if (
            args.vision_question is not None
            and not args.vision_question.strip()
        ):
            parser.error("--vision-question 不能是空字符串")
        if args.vision_image is not None and args.vision_save_frame is not None:
            parser.error(
                "--vision-save-frame 只保存摄像头捕获帧，"
                "不能与 --vision-image 同时使用"
            )
        try:
            vision_service = build_vision_service_from_args(
                args=args,
                environ=environment,
                client_builder=vlm_client_builder,
                service_builder=vision_service_builder,
            )
        except (ValueError, VisionError) as error:
            output_func(f"视觉配置错误：{error}")
            return 2
        try:
            return run_vision_command_loop(
                vision_service,
                image_path=args.vision_image,
                question=args.vision_question,
                save_frame_path=args.vision_save_frame,
                output_format=args.vision_output_format,
                input_func=input_func,
                output_func=output_func,
            )
        except KeyboardInterrupt:
            output_func("\n收到 Ctrl+C，退出视觉模式；未创建机械臂 Runtime。")
            return 130

    if (
        args.backend == "so100_plus"
        and not args.acknowledge_so100_plus_risk
    ):
        parser.error(
            "真机模式必须同时传入 --acknowledge-so100-plus-risk；"
            "该模式会连接电机、启用力矩，并允许 Skill 产生真实运动。"
        )

    llm_client: LLMClient | None = None
    rag_context_provider: RagContextProvider | None = None
    if args.input_mode == "llm":
        try:
            llm_client = build_llm_client_from_environment(
                environ=environment,
                client_builder=llm_client_builder,
            )
        except (ValueError, LLMClientError) as error:
            output_func(f"LLM 配置错误: {error}")
            return 2
        if not args.disable_rag:
            try:
                rag_context_provider = rag_context_provider_builder(
                    knowledge_directory=args.knowledge_dir,
                    top_k=args.rag_top_k,
                )
            except Exception as error:
                output_func(
                    "RAG 初始化失败，已退回基础 Command Prompt："
                    f"{error}"
                )

    try:
        runtime = runtime_builder(args)
    except KeyboardInterrupt:
        output_func("\n真机装配被 Ctrl+C 中断；已执行装配失败清理。")
        return 130
    except Exception as error:
        output_func(f"启动失败: {error}")
        return 2

    exit_code = 0
    shutdown_error: Exception | None = None
    emergency_exit_requested = False
    try:
        output_func(f"当前后端: {args.backend}")
        session_state = getattr(runtime, "session_state", None)
        if session_state is not None:
            output_func(f"SO-100 Plus 会话状态: {session_state.value}")
            if session_state.value == "REST":
                output_func(
                    "已认证为 follower_rest；可执行 unfold_arm，"
                    "普通 move_arm 保持禁用。"
                )
            elif session_state.value == "WORK":
                output_func(
                    "已认证为不规则 WORK 空间的 middle_internal；"
                    "目标将优先从当前姿态直达并经过完整轨迹门禁，"
                    "必要时才回退到 middle_internal 中心通道。"
                )
        if runtime.current_tcp_position_m is not None:
            position = ", ".join(
                f"{value:.6f}"
                for value in runtime.current_tcp_position_m
            )
            output_func(f"启动 TCP (m): {position}")
        if runtime.move_arm_disabled_reason is not None:
            output_func(runtime.move_arm_disabled_reason)
        if args.input_mode == "llm":
            generator = CommandGenerator(
                client=llm_client,
                skills=runtime.skills,
                context_provider=rag_context_provider,
                runtime_state_provider=lambda: _runtime_state_for_prompt(
                    runtime, args.backend
                ),
                event_handler=output_func,
                max_context_chars=args.rag_max_context_chars,
            )
            emergency_exit_requested = run_llm_command_loop(
                runtime,
                generator,
                input_func=input_func,
                output_func=output_func,
            )
        else:
            emergency_exit_requested = run_json_command_loop(
                runtime,
                input_func=input_func,
                output_func=output_func,
            )
    except KeyboardInterrupt:
        output_func(
            "\n收到 Ctrl+C，将按紧急退出处理：停止、关闭力矩并断开；"
            "机械臂可能未处于 REST。"
        )
        emergency_exit_requested = True
        exit_code = 130
    finally:
        if not emergency_exit_requested:
            exit_pose_warning = getattr(runtime, "exit_pose_warning", None)
            if exit_pose_warning is not None:
                output_func(exit_pose_warning)
        try:
            _shutdown_runtime(
                runtime,
                emergency=emergency_exit_requested,
            )
        except Exception as error:
            shutdown_error = error

    if shutdown_error is not None:
        output_func(f"停止或断开失败: {shutdown_error}")
        return 1

    if getattr(runtime, "torque_disabled_on_shutdown", False):
        output_func("机械臂已停止，力矩已关闭，后端连接已断开。")
    else:
        output_func("机械臂已停止，后端连接已断开；未自动关闭力矩。")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
