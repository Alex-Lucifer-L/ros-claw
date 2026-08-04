"""使用现有 LLM 客户端把单轮自然语言转换为 Command。"""

from collections.abc import Callable
from uuid import uuid4

from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.llm.client import LLMClient
from rosclaw_mini.llm.command_parser import parse_json_command
from rosclaw_mini.llm.prompt_builder import build_command_prompt
from rosclaw_mini.rag.context import (
    DEFAULT_RAG_MAX_CONTEXT_CHARS,
    RagContextProvider,
    describe_retrieval,
)
from rosclaw_mini.skills.base import SkillDefinition


class CommandGenerator:
    def __init__(
        self,
        client: LLMClient,
        skills: dict[str, SkillDefinition],
        *,
        context_provider: RagContextProvider | None = None,
        runtime_state_provider: Callable[[], str | None] | None = None,
        event_handler: Callable[[str], None] | None = None,
        max_context_chars: int = DEFAULT_RAG_MAX_CONTEXT_CHARS,
    ) -> None:
        self.client = client
        self.skills = skills
        self.context_provider = context_provider
        self.runtime_state_provider = runtime_state_provider
        self.event_handler = event_handler
        self.max_context_chars = max_context_chars

    def _report(self, message: str) -> None:
        if self.event_handler is not None:
            self.event_handler(message)

    def generate(self, user_input: str) -> Command:
        retrieved_chunks = None
        if self.context_provider is not None:
            try:
                retrieved_chunks = self.context_provider.retrieve(user_input)
            except Exception as error:
                self._report(
                    "RAG 检索失败，已退回基础 Command Prompt："
                    f"{error}"
                )
            else:
                self._report(describe_retrieval(retrieved_chunks))

        runtime_state = None
        if self.runtime_state_provider is not None:
            try:
                runtime_state = self.runtime_state_provider()
            except Exception as error:
                self._report(
                    "运行时状态读取失败，本次 Prompt 不加入实时状态："
                    f"{error}"
                )

        prompt = build_command_prompt(
            user_input=user_input,
            skills=self.skills,
            retrieved_chunks=retrieved_chunks,
            runtime_state=runtime_state,
            max_context_chars=self.max_context_chars,
        )

        model_response = self.client.generate(prompt)

        command_id = str(uuid4())

        return parse_json_command(
            command_json=model_response,
            command_id=command_id,
        )
