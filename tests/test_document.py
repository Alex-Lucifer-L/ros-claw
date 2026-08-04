from dataclasses import FrozenInstanceError

import pytest

from rosclaw_mini.rag.document import (
    KnowledgeChunk,
    KnowledgeDocument,
    RetrievedChunk,
)


def test_knowledge_document_stores_fields() -> None:
    document = KnowledgeDocument(
        document_id="session-states",
        title="机械臂会话状态",
        category="state",
        content="REST、WORK 和 UNVERIFIED 状态说明。",
        source="project-documentation",
        version="1.0",
        risk_level="high",
    )

    assert document.document_id == "session-states"
    assert document.title == "机械臂会话状态"
    assert document.category == "state"
    assert document.content == "REST、WORK 和 UNVERIFIED 状态说明。"
    assert document.source == "project-documentation"
    assert document.version == "1.0"
    assert document.risk_level == "high"
    assert document.metadata == {}


def test_document_metadata_is_not_shared() -> None:
    first = KnowledgeDocument(
        document_id="first",
        title="第一篇文档",
        category="test",
        content="第一篇文档的内容。",
        source="test",
        version="1.0",
        risk_level="low",
    )

    second = KnowledgeDocument(
        document_id="second",
        title="第二篇文档",
        category="test",
        content="第二篇文档的内容。",
        source="test",
        version="1.0",
        risk_level="low",
    )

    assert first.metadata is not second.metadata


def test_knowledge_document_cannot_reassign_fields() -> None:
    document = KnowledgeDocument(
        document_id="session-states",
        title="机械臂会话状态",
        category="state",
        content="状态说明。",
        source="project-documentation",
        version="1.0",
        risk_level="high",
    )

    with pytest.raises(FrozenInstanceError):
        document.title = "被修改的标题"


def test_knowledge_chunk_keeps_document_relationship() -> None:
    chunk = KnowledgeChunk(
        chunk_id="session-states-0",
        document_id="session-states",
        chunk_index=0,
        content="从 REST 进入 WORK 应使用 unfold_arm。",
        metadata={"section": "REST 到 WORK"},
    )

    assert chunk.chunk_id == "session-states-0"
    assert chunk.document_id == "session-states"
    assert chunk.chunk_index == 0
    assert chunk.content == "从 REST 进入 WORK 应使用 unfold_arm。"
    assert chunk.metadata == {"section": "REST 到 WORK"}


def test_retrieved_chunk_wraps_chunk_with_score() -> None:
    chunk = KnowledgeChunk(
        chunk_id="session-states-0",
        document_id="session-states",
        chunk_index=0,
        content="从 REST 进入 WORK 应使用 unfold_arm。",
    )

    result = RetrievedChunk(
        chunk=chunk,
        score=0.92,
    )

    assert result.chunk is chunk
    assert result.chunk.content == "从 REST 进入 WORK 应使用 unfold_arm。"
    assert result.score == pytest.approx(0.92)