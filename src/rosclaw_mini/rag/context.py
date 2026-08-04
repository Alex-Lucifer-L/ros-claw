"""项目知识库的统一加载、检索和 Prompt 上下文格式化入口。"""

from collections.abc import Sequence
from pathlib import Path

from rosclaw_mini.rag.chunker import chunk_markdown_document
from rosclaw_mini.rag.document import (
    KnowledgeChunk,
    KnowledgeDocument,
    RetrievedChunk,
)
from rosclaw_mini.rag.loader import load_markdown_documents
from rosclaw_mini.rag.retriever import KeywordRetriever, Retriever


DEFAULT_RAG_TOP_K = 4
DEFAULT_RAG_MAX_CONTEXT_CHARS = 6000


class RagContextProvider:
    """一次加载项目知识，并为每条自然语言命令检索少量上下文。"""

    def __init__(
        self,
        documents: Sequence[KnowledgeDocument],
        *,
        top_k: int = DEFAULT_RAG_TOP_K,
        retriever: Retriever | None = None,
    ) -> None:
        if top_k <= 0:
            raise ValueError("RAG top_k 必须大于 0")
        self.documents = tuple(documents)
        self.chunks = tuple(
            chunk
            for document in self.documents
            for chunk in chunk_markdown_document(document)
        )
        self.top_k = top_k
        self.retriever = (
            retriever
            if retriever is not None
            else KeywordRetriever(self.chunks)
        )

    @classmethod
    def from_directory(
        cls,
        directory: str | Path,
        *,
        top_k: int = DEFAULT_RAG_TOP_K,
    ) -> "RagContextProvider":
        return cls(load_markdown_documents(directory), top_k=top_k)

    def retrieve(self, query: str) -> list[RetrievedChunk]:
        return self.retriever.retrieve(query, top_k=self.top_k)


def _source_name(result: RetrievedChunk) -> str:
    section = str(result.chunk.metadata.get("section", "未命名章节"))
    section = " ".join(section.split())
    return f"{result.chunk.document_id}#{section}"


def format_retrieved_knowledge(
    results: Sequence[RetrievedChunk],
    *,
    max_chars: int = DEFAULT_RAG_MAX_CONTEXT_CHARS,
) -> str:
    """生成带来源边界的受限上下文，绝不无条件塞入全部知识。"""

    if max_chars < 128:
        raise ValueError("RAG 上下文字符上限不能小于 128")

    opening = "[PROJECT_KNOWLEDGE]\n"
    closing = "\n[/PROJECT_KNOWLEDGE]"
    if not results:
        return opening + "未检索到匹配的项目知识。" + closing

    remaining = max_chars - len(opening) - len(closing)
    blocks: list[str] = []
    for result in results:
        source_name = _source_name(result)
        prefix = (
            f"[SOURCE: {source_name}]\n"
            f"[CHUNK: {result.chunk.chunk_index} | "
            f"SCORE: {result.score:.4f}]\n"
        )
        source_files = result.chunk.metadata.get("source_files", [])
        source_line = ""
        if source_files:
            source_line = (
                "[SOURCE_FILES: "
                + ", ".join(str(item) for item in source_files)
                + "]\n"
            )
        suffix = "\n[/SOURCE]"
        separator_length = 2 if blocks else 0
        fixed_length = (
            len(prefix) + len(source_line) + len(suffix) + separator_length
        )
        if fixed_length >= remaining:
            break

        content_budget = remaining - fixed_length
        content = result.chunk.content
        if len(content) > content_budget:
            if content_budget <= 1:
                break
            content = content[: content_budget - 1].rstrip() + "…"
        block = prefix + source_line + content + suffix
        blocks.append(block)
        remaining -= len(block) + separator_length

        if len(content) < len(result.chunk.content):
            break

    if not blocks:
        blocks.append("知识结果因上下文上限过小而未加入。")
    return opening + "\n\n".join(blocks) + closing


def describe_retrieval(results: Sequence[RetrievedChunk]) -> str:
    """生成不含 Prompt、密钥和完整用户输入的可审计检索摘要。"""

    if not results:
        return "RAG 检索未命中项目知识，使用基础 Command Prompt。"
    items = ", ".join(
        f"{_source_name(result)}(score={result.score:.4f})"
        for result in results
    )
    return f"RAG 检索命中：{items}"
