from uuid import uuid4

from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.llm.client import LLMClient
from rosclaw_mini.llm.command_parser import parse_json_command
from rosclaw_mini.llm.prompt_builder import build_command_prompt
from rosclaw_mini.skills.base import SkillDefinition


class CommandGenerator:
    def __init__(
        self,
        client: LLMClient,
        skills: dict[str, SkillDefinition],
    ) -> None:
        self.client = client
        self.skills = skills

    def generate(self, user_input: str) -> Command:
        prompt = build_command_prompt(
            user_input=user_input,
            skills=self.skills,
        )

        model_response = self.client.generate(prompt)

        command_id = str(uuid4())

        return parse_json_command(
            command_json=model_response,
            command_id=command_id,
        )