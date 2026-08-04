from __future__ import annotations

import base64

import pytest

from rosclaw_mini.llm.client import LLMClientError
from rosclaw_mini.vision.exceptions import (
    VLMConfigurationError,
    VLMRequestError,
)
from rosclaw_mini.vision.vlm_client import QwenVLMClient


class FakeChatClient:
    def __init__(self, result='{"scene_description":"ok","objects":[]}'):
        self.result = result
        self.messages = None

    def generate_messages(self, messages):
        self.messages = messages
        if isinstance(self.result, Exception):
            raise self.result
        return self.result


def test_qwen_vlm_client_builds_compatible_multimodal_message():
    chat = FakeChatClient()
    client = QwenVLMClient(
        api_key="placeholder-key",
        model="fake-vl",
        chat_client=chat,
    )
    result = client.generate(
        image_bytes=b"small-image",
        mime_type="image/jpeg",
        prompt="describe",
    )

    expected = base64.b64encode(b"small-image").decode("ascii")
    assert result.startswith("{")
    assert chat.messages[0]["content"][0]["image_url"]["url"] == (
        f"data:image/jpeg;base64,{expected}"
    )
    assert chat.messages[0]["content"][1] == {
        "type": "text",
        "text": "describe",
    }


def test_qwen_vlm_client_requires_api_key():
    with pytest.raises(VLMConfigurationError, match="API Key"):
        QwenVLMClient(api_key="", model="fake-vl", chat_client=FakeChatClient())


def test_qwen_vlm_client_maps_timeout_without_leaking_secrets_or_image():
    chat = FakeChatClient(LLMClientError("调用 LLM 服务超时"))
    client = QwenVLMClient(
        api_key="super-secret-key",
        model="fake-vl",
        chat_client=chat,
    )
    with pytest.raises(VLMRequestError) as caught:
        client.generate(
            image_bytes=b"private-image-bytes",
            mime_type="image/jpeg",
            prompt="describe",
        )
    message = str(caught.value)
    assert "超时" in message
    assert "super-secret-key" not in message
    assert "private-image-bytes" not in message
    assert "base64" not in message

