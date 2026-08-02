from io import BytesIO
import json
from urllib.error import HTTPError, URLError

import pytest

import rosclaw_mini.llm.openai_compatible_client as client_module
from rosclaw_mini.llm.client import LLMClientError
from rosclaw_mini.llm.openai_compatible_client import OpenAICompatibleClient


class FakeHTTPResponse:
    def __init__(self, body: bytes) -> None:
        self._body = body

    def __enter__(self):
        return self

    def __exit__(self, _error_type, _error, _traceback) -> None:
        return None

    def read(self) -> bytes:
        return self._body


def test_client_sends_compatible_request_and_parses_content(monkeypatch):
    captured = {}

    def fake_urlopen(request, *, timeout):
        captured["request"] = request
        captured["timeout"] = timeout
        return FakeHTTPResponse(
            json.dumps(
                {"choices": [{"message": {"content": "  generated JSON  "}}]}
            ).encode("utf-8")
        )

    monkeypatch.setattr(client_module, "urlopen", fake_urlopen)
    client = OpenAICompatibleClient(
        base_url=" http://localhost:11434/v1/ ",
        model=" qwen-test ",
        api_key=" test-placeholder ",
        timeout_seconds=12.5,
    )

    result = client.generate("请生成命令")

    request = captured["request"]
    payload = json.loads(request.data.decode("utf-8"))
    assert result == "generated JSON"
    assert request.full_url == "http://localhost:11434/v1/chat/completions"
    assert request.get_header("Authorization") == "Bearer test-placeholder"
    assert payload == {
        "model": "qwen-test",
        "messages": [{"role": "user", "content": "请生成命令"}],
        "stream": False,
    }
    assert captured["timeout"] == 12.5


def test_client_omits_authorization_without_api_key(monkeypatch):
    captured = {}

    def fake_urlopen(request, *, timeout):
        captured["request"] = request
        return FakeHTTPResponse(
            b'{"choices":[{"message":{"content":"ok"}}]}'
        )

    monkeypatch.setattr(client_module, "urlopen", fake_urlopen)

    client = OpenAICompatibleClient(
        base_url="http://localhost:11434/v1",
        model="local-model",
    )
    assert client.generate("prompt") == "ok"
    assert captured["request"].get_header("Authorization") is None


def test_client_converts_http_error(monkeypatch):
    def raise_http_error(_request, *, timeout):
        raise HTTPError(
            url="http://example.test/v1/chat/completions",
            code=401,
            msg="Unauthorized",
            hdrs=None,
            fp=BytesIO(b'{"error":"unauthorized"}'),
        )

    monkeypatch.setattr(client_module, "urlopen", raise_http_error)
    client = OpenAICompatibleClient(
        base_url="http://example.test/v1",
        model="test-model",
    )

    with pytest.raises(LLMClientError, match="HTTP 401"):
        client.generate("prompt")


def test_client_converts_network_error(monkeypatch):
    def raise_network_error(_request, *, timeout):
        raise URLError("connection refused")

    monkeypatch.setattr(client_module, "urlopen", raise_network_error)
    client = OpenAICompatibleClient(
        base_url="http://example.test/v1",
        model="test-model",
    )

    with pytest.raises(LLMClientError, match="无法连接 LLM 服务"):
        client.generate("prompt")


def test_client_converts_timeout(monkeypatch):
    def raise_timeout(_request, *, timeout):
        raise TimeoutError("timed out")

    monkeypatch.setattr(client_module, "urlopen", raise_timeout)
    client = OpenAICompatibleClient(
        base_url="http://example.test/v1",
        model="test-model",
    )

    with pytest.raises(LLMClientError, match="超时"):
        client.generate("prompt")


@pytest.mark.parametrize(
    ("body", "error_pattern"),
    (
        (b"not-json", "不是合法 JSON"),
        (b"{}", "缺少 choices"),
        (b'{"choices":[]}', "缺少 choices"),
        (
            b'{"choices":[{"message":{"content":123}}]}',
            "message.content 不是字符串",
        ),
        (
            b'{"choices":[{"message":{"content":"   "}}]}',
            "空内容",
        ),
    ),
)
def test_client_converts_invalid_responses(
    monkeypatch,
    body,
    error_pattern,
):
    monkeypatch.setattr(
        client_module,
        "urlopen",
        lambda _request, *, timeout: FakeHTTPResponse(body),
    )
    client = OpenAICompatibleClient(
        base_url="http://example.test/v1",
        model="test-model",
    )

    with pytest.raises(LLMClientError, match=error_pattern):
        client.generate("prompt")


@pytest.mark.parametrize(
    "kwargs",
    (
        {"base_url": "", "model": "model"},
        {"base_url": "localhost:11434/v1", "model": "model"},
        {"base_url": "http:///v1", "model": "model"},
        {"base_url": "http://localhost/v1?query=1", "model": "model"},
        {"base_url": "http://localhost/v1", "model": ""},
        {
            "base_url": "http://localhost/v1",
            "model": "model",
            "timeout_seconds": 0,
        },
        {
            "base_url": "http://localhost/v1",
            "model": "model",
            "timeout_seconds": float("inf"),
        },
    ),
)
def test_client_rejects_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        OpenAICompatibleClient(**kwargs)


@pytest.mark.parametrize("prompt", ("", "   ", None))
def test_client_rejects_empty_prompt(prompt):
    client = OpenAICompatibleClient(
        base_url="http://localhost/v1",
        model="model",
    )

    with pytest.raises(ValueError, match="prompt"):
        client.generate(prompt)
