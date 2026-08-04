"""RAG 知识文档、分块和检索结果的数据结构。"""

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class KnowledgeDocument:
    """
    表示一个知识文档。

    Attributes:
        document_id (str): 文档的唯一标识符。
        title (str): 文档的标题。
        category (str): 文档的分类。
        content (str): 文档的内容。
        source (str): 文档的来源。
        version (str): 文档的版本号。
        risk_level (str): 文档的风险等级。
        metadata (dict[str, Any]): 其他元数据，默认为空字典。
    """
    document_id: str
    title: str
    category: str
    content: str
    source: str
    version: str
    risk_level: str
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class KnowledgeChunk:
    """
    表示一个知识文档的分块。

    Attributes:
        chunk_id (str): 分块的唯一标识符。
        document_id (str): 所属文档的唯一标识符。
        chunk_index (int): 分块在文档中的索引位置。
        content (str): 分块的内容。
        metadata (dict[str, Any]): 其他元数据，默认为空字典。
    """
    chunk_id: str
    document_id: str
    chunk_index: int
    content: str
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RetrievedChunk:
    """
    表示一个检索到的知识文档分块。

    Attributes:
        chunk (KnowledgeChunk): 检索到的知识文档分块。
        score (float): 检索结果的评分。
    """
    chunk: KnowledgeChunk
    score: float
