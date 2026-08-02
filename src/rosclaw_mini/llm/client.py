from typing import Protocol


class LLMClientError(RuntimeError):
    """调用 LLM 服务或解析服务响应时发生的错误。"""


class LLMClient(Protocol):
    def generate(self, prompt: str) -> str:
        """
        接收完整提示词，返回模型生成的文本。
        """
        ...
