from dataclasses import dataclass
import json
import math
from numbers import Real
from socket import timeout as SocketTimeout
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from rosclaw_mini.llm.client import LLMClientError


@dataclass(frozen=True)
class OpenAICompatibleClient:
    base_url: str
    model: str
    api_key: str | None = None
    timeout_seconds: float = 30.0

    def __post_init__(self) -> None:
        if not isinstance(self.base_url, str) or not self.base_url.strip():
            raise ValueError("base_url 不能为空")
        base_url = self.base_url.strip().rstrip("/")
        parsed_url = urlsplit(base_url)
        if parsed_url.scheme not in {"http", "https"} or not parsed_url.netloc:
            raise ValueError(
                "base_url 必须是包含主机名的 http:// 或 https:// URL"
            )
        if parsed_url.query or parsed_url.fragment:
            raise ValueError("base_url 不能包含查询参数或片段")
        if not isinstance(self.model, str) or not self.model.strip():
            raise ValueError("model 不能为空")
        if (
            isinstance(self.timeout_seconds, bool)
            or not isinstance(self.timeout_seconds, Real)
            or not math.isfinite(float(self.timeout_seconds))
            or float(self.timeout_seconds) <= 0
        ):
            raise ValueError("timeout_seconds 必须是有限正数")
        if self.api_key is not None and not isinstance(self.api_key, str):
            raise ValueError("api_key 必须是字符串或 None")

        object.__setattr__(self, "base_url", base_url)
        object.__setattr__(self, "model", self.model.strip())
        object.__setattr__(
            self,
            "api_key",
            self.api_key.strip() if self.api_key and self.api_key.strip() else None,
        )
        object.__setattr__(self, "timeout_seconds", float(self.timeout_seconds))

    def generate(self, prompt: str) -> str:
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("prompt 不能为空")

        endpoint = (
            f"{self.base_url.rstrip('/')}/chat/completions"
        )

        request_body = {
            "model": self.model,
            "messages": [
                {
                    "role": "user",
                    "content": prompt,
                }
            ],
            "stream": False,
        }

        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
        }

        if self.api_key:
            headers["Authorization"] = (
                f"Bearer {self.api_key}"
            )

        request = Request(
            url=endpoint,
            data=json.dumps(request_body).encode("utf-8"),
            headers=headers,
            method="POST",
        )

        try:
            with urlopen(
                request,
                timeout=self.timeout_seconds,
            ) as response:
                response_text = (
                    response.read().decode(
                        "utf-8",
                        errors="replace",
                    )
                )

        except HTTPError as error:
            error_body = (
                error.read().decode(
                    "utf-8",
                    errors="replace",
                )
            )

            raise LLMClientError(
                f"LLM 请求失败，HTTP {error.code}: "
                f"{error_body[:500]}"
            ) from error

        except URLError as error:
            if isinstance(error.reason, (TimeoutError, SocketTimeout)):
                raise LLMClientError("调用 LLM 服务超时") from error
            raise LLMClientError(
                f"无法连接 LLM 服务: {error.reason}"
            ) from error

        except (TimeoutError, SocketTimeout) as error:
            raise LLMClientError(
                "调用 LLM 服务超时"
            ) from error

        except OSError as error:
            raise LLMClientError(
                f"调用 LLM 服务时发生网络错误: {error}"
            ) from error

        try:
            response_data = json.loads(response_text)
        except json.JSONDecodeError as error:
            raise LLMClientError(
                "LLM 服务返回的不是合法 JSON 响应"
            ) from error

        try:
            content = (
                response_data["choices"][0]
                ["message"]["content"]
            )
        except (KeyError, IndexError, TypeError) as error:
            raise LLMClientError(
                "LLM 服务响应中缺少 "
                "choices[0].message.content"
            ) from error

        if not isinstance(content, str):
            raise LLMClientError(
                "LLM 服务返回的 message.content 不是字符串"
            )

        content = content.strip()

        if not content:
            raise LLMClientError(
                "LLM 服务返回了空内容"
            )

        return content
