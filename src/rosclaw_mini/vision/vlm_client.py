"""Alibaba Bailian/OpenAI-compatible Qwen vision client."""

from __future__ import annotations

import base64
import math
from numbers import Real
from typing import Protocol

from rosclaw_mini.llm.client import LLMClientError
from rosclaw_mini.llm.openai_compatible_client import OpenAICompatibleClient
from rosclaw_mini.vision.exceptions import VLMConfigurationError, VLMRequestError


DEFAULT_DASHSCOPE_BASE_URL = (
    "https://dashscope.aliyuncs.com/compatible-mode/v1"
)
DEFAULT_QWEN_VL_MODEL = "qwen-vl-plus"
SUPPORTED_IMAGE_MIME_TYPES = frozenset(
    {"image/jpeg", "image/png", "image/webp"}
)


class MultimodalChatClient(Protocol):
    def generate_messages(self, messages: list[dict]) -> str:
        ...


class VLMClient(Protocol):
    @property
    def model(self) -> str:
        ...

    def generate(
        self,
        *,
        image_bytes: bytes,
        mime_type: str,
        prompt: str,
    ) -> str:
        ...


class QwenVLMClient:
    """Send one image and one prompt through the compatible chat endpoint."""

    def __init__(
        self,
        *,
        base_url: str = DEFAULT_DASHSCOPE_BASE_URL,
        model: str = DEFAULT_QWEN_VL_MODEL,
        api_key: str,
        timeout_seconds: float = 30.0,
        chat_client: MultimodalChatClient | None = None,
    ) -> None:
        if not isinstance(api_key, str) or not api_key.strip():
            raise VLMConfigurationError(
                "视觉模式缺少 API Key；请设置 ROSCLAW_LLM_API_KEY "
                "或 DASHSCOPE_API_KEY。"
            )
        if not isinstance(model, str) or not model.strip():
            raise VLMConfigurationError("视觉模型名称不能为空。")
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, Real)
            or not math.isfinite(float(timeout_seconds))
            or float(timeout_seconds) <= 0
        ):
            raise VLMConfigurationError("视觉请求超时时间必须是有限正数。")

        self._model = model.strip()
        if chat_client is None:
            try:
                chat_client = OpenAICompatibleClient(
                    base_url=base_url,
                    model=self._model,
                    api_key=api_key.strip(),
                    timeout_seconds=float(timeout_seconds),
                )
            except ValueError as error:
                raise VLMConfigurationError(
                    f"视觉模型配置无效：{error}"
                ) from error
        self._chat_client = chat_client

    @property
    def model(self) -> str:
        return self._model

    def generate(
        self,
        *,
        image_bytes: bytes,
        mime_type: str,
        prompt: str,
    ) -> str:
        if not isinstance(image_bytes, bytes) or not image_bytes:
            raise VLMConfigurationError("视觉请求的图像数据不能为空。")
        if mime_type not in SUPPORTED_IMAGE_MIME_TYPES:
            raise VLMConfigurationError(
                f"不支持的图像 MIME 类型：{mime_type!r}。"
            )
        if not isinstance(prompt, str) or not prompt.strip():
            raise VLMConfigurationError("视觉 Prompt 不能为空。")

        encoded = base64.b64encode(image_bytes).decode("ascii")
        messages = [
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:{mime_type};base64,{encoded}"
                        },
                    },
                    {"type": "text", "text": prompt.strip()},
                ],
            }
        ]
        try:
            return self._chat_client.generate_messages(messages)
        except LLMClientError as error:
            # Do not include request bodies, encoded images or credentials.
            raise VLMRequestError(f"视觉模型调用失败：{error}") from error
        except (OSError, TimeoutError) as error:
            raise VLMRequestError(
                f"视觉模型网络请求失败：{type(error).__name__}"
            ) from error

