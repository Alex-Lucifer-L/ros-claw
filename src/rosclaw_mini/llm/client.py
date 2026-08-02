from typing import Protocol


class LLMClient(Protocol):
    def generate(self, prompt: str) -> str:
        """
        调用LLM，生成统一的JSON格式的指令。该方法接受一个字符串类型的 prompt 参数，表示输入的提示信息，并返回一个字符串类型的结果，表示生成的JSON格式指令。
        """
        ...