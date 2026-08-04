import re
from collections.abc import Sequence

from rosclaw_mini.rag.document import (
    KnowledgeChunk,
    RetrievedChunk,
)


TOKEN_PATTERN = re.compile(
    r"[a-zA-Z0-9_]+|[\u4e00-\u9fff]"
)


def _tokenize(text: str) -> set[str]:
    """
    将文本转换成用于关键词匹配的词元集合。

    英文和数字按完整单词提取，例如 REST、move_arm。
    中文暂时按单个汉字提取。
    """
    return {
        match.group(0).lower()
        for match in TOKEN_PATTERN.finditer(text)
    }


class KeywordRetriever:
    def __init__(
        self,
        chunks: Sequence[KnowledgeChunk],
    ) -> None:
        self._chunks = tuple(chunks)

    def retrieve(
        self,
        query: str,
        top_k: int = 3,
    ) -> list[RetrievedChunk]:
        if top_k <= 0:
            raise ValueError("top_k 必须大于 0")

        query_tokens = _tokenize(query)

        if not query_tokens:
            return []

        results: list[RetrievedChunk] = []

        for chunk in self._chunks:
            metadata_text = " ".join(
                str(value)
                for value in chunk.metadata.values()
            )

            searchable_text = (
                f"{chunk.content}\n{metadata_text}"
            )

            chunk_tokens = _tokenize(searchable_text)
            matched_tokens = query_tokens & chunk_tokens

            if not matched_tokens:
                continue

            score = len(matched_tokens) / len(query_tokens)

            results.append(
                RetrievedChunk(
                    chunk=chunk,
                    score=score,
                )
            )

        results.sort(
            key=lambda result: (
                -result.score,
                result.chunk.chunk_index,
            )
        )

        return results[:top_k]