"""RosClaw Mini 的 JSON 命令行入口。"""

import argparse
from collections.abc import Callable, Mapping, Sequence
import json
import os
from pathlib import Path
import uuid
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.llm.client import LLMClient, LLMClientError
from rosclaw_mini.llm.command_generator import CommandGenerator
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


InputFunction = Callable[[str], str]###定义了一个类型别名 InputFunction，它表示一个可调用对象（函数或方法），该对象接受一个字符串参数并返回一个字符串。这个类型别名用于表示输入函数的签名，通常用于从用户获取输入。
OutputFunction = Callable[[str], None]###定义了一个类型别名 OutputFunction，它表示一个可调用对象（函数或方法），该对象接受一个字符串参数并返回 None。这个类型别名用于表示输出函数的签名，通常用于向用户显示输出信息。
RuntimeBuilder = Callable[[argparse.Namespace], ArmRuntime]###定义了一个类型别名 RuntimeBuilder，它表示一个可调用对象（函数或方法），该对象接受一个 argparse.Namespace 对象作为参数并返回一个 ArmRuntime 对象。这个类型别名用于表示运行时构建器的签名，通常用于根据命令行参数创建和配置 ArmRuntime 实例。
LLMClientBuilder = Callable[..., LLMClient]
RagContextProviderBuilder = Callable[..., RagContextProvider]

DEFAULT_KNOWLEDGE_DIRECTORY = Path(__file__).resolve().parents[2] / "knowledge"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="运行 RosClaw Mini 输入 → Gateway → Skill 命令链路。",
    )
    parser.add_argument(
        "--input-mode",
        choices=("json", "llm"),
        default="json",
        help="输入模式；默认 json，llm 使用 OpenAI-compatible 服务。",
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


def _runtime_state_for_prompt(runtime: ArmRuntime, backend: str) -> str:
    session_state = getattr(runtime, "session_state", None)
    if session_state is None:
        return f"backend={backend}; session_state=NOT_APPLICABLE"
    state_value = getattr(session_state, "value", str(session_state))
    return f"backend={backend}; session_state={state_value}"

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

    return (
        "当前有命令正在执行，"
        "请等待其完成或使用 stop 命令停止它。"
    )

def run_json_command_loop(
    runtime: ArmRuntime,
    *,
    input_func: InputFunction = input,
    output_func: OutputFunction = print,
) -> None:
    """读取 JSON 命令，并交给现有 Gateway 与 ExecutionController。"""

    while True:
        try:
            command_text = input_func("请输入指令：")
        except EOFError:
            output_func("输入已结束，准备退出程序。")
            return

        command_text = command_text.strip()
        if command_text == "exit":
            output_func("准备退出程序。")
            return

        if command_text == "result":
            if runtime.controller.is_running():
                output_func("命令正在执行中，请等待其完成。")
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

        output_func(dispatch_command(runtime, command))

def run_llm_command_loop(
    runtime: ArmRuntime,
    generator: CommandGenerator,
    *,
    input_func: InputFunction = input,
    output_func: OutputFunction = print,
) -> None:
    """读取自然语言，通过 LLM 生成 Command 后进入现有执行链。"""

    while True:
        try:
            user_input = input_func("请输入自然语言指令：")
        except EOFError:
            output_func("输入已结束，准备退出程序。")
            return

        user_input = user_input.strip()

        if user_input == "exit":
            output_func("准备退出程序。")
            return

        if user_input == "result":
            if runtime.controller.is_running():
                output_func("命令正在执行中，请等待其完成。")
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

        output_func(dispatch_command(runtime, command))


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
    environ: Mapping[str, str] | None = None,
) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.rag_top_k <= 0:
        parser.error("--rag-top-k 必须大于 0")
    if args.rag_max_context_chars < 128:
        parser.error("--rag-max-context-chars 不能小于 128")
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
                environ=os.environ if environ is None else environ,
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
                    "目标将经过网格单元、中心通道和完整轨迹门禁。"
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
            run_llm_command_loop(
                runtime,
                generator,
                input_func=input_func,
                output_func=output_func,
            )
        else:
            run_json_command_loop(
                runtime,
                input_func=input_func,
                output_func=output_func,
            )
    except KeyboardInterrupt:
        output_func("\n收到 Ctrl+C，正在停止并断开连接。")
        exit_code = 130
    finally:
        exit_pose_warning = getattr(runtime, "exit_pose_warning", None)
        if exit_pose_warning is not None:
            output_func(exit_pose_warning)
        try:
            runtime.shutdown()
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
