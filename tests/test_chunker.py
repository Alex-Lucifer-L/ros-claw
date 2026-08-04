from rosclaw_mini.rag.chunker import chunk_markdown_document
from rosclaw_mini.rag.document import KnowledgeDocument


def make_document(content: str) -> KnowledgeDocument:
    return KnowledgeDocument(
        document_id="session-states",
        title="机械臂会话状态",
        category="state",
        content=content,
        source="rosclaw-mini-project",
        version="1.0",
        risk_level="high",
        metadata={
            "tags": ["state", "safety", "workflow"],
        },
    )


def test_chunk_markdown_document_splits_by_second_level_heading() -> None:
    document = make_document(
        """# 机械臂会话状态

机械臂会话状态的总体说明。

## REST

REST 表示机械臂处于休息状态。

## WORK

WORK 表示机械臂处于工作状态。
"""
    )

    chunks = chunk_markdown_document(document)

    assert len(chunks) == 3

    assert chunks[0].chunk_id == "session-states-0"
    assert chunks[0].metadata["section"] == "机械臂会话状态"

    assert chunks[1].chunk_id == "session-states-1"
    assert chunks[1].metadata["section"] == "REST"

    assert chunks[2].chunk_id == "session-states-2"
    assert chunks[2].metadata["section"] == "WORK"


def test_chunk_indices_follow_original_order() -> None:
    document = make_document(
        """## REST

REST 状态说明。

## TRANSITION

TRANSITION 状态说明。

## WORK

WORK 状态说明。
"""
    )

    chunks = chunk_markdown_document(document)

    assert [chunk.chunk_index for chunk in chunks] == [0, 1, 2]
    assert [chunk.metadata["section"] for chunk in chunks] == [
        "REST",
        "TRANSITION",
        "WORK",
    ]


def test_chunk_content_keeps_heading_and_text() -> None:
    document = make_document(
        """## REST

REST 表示机械臂处于收纳或休息状态。
"""
    )

    chunks = chunk_markdown_document(document)

    assert len(chunks) == 1
    assert chunks[0].content == (
        "## REST\n\n"
        "REST 表示机械臂处于收纳或休息状态。"
    )


def test_chunk_inherits_document_metadata() -> None:
    document = make_document(
        """## REST

REST 状态说明。

## WORK

WORK 状态说明。
"""
    )

    chunks = chunk_markdown_document(document)

    rest_chunk = chunks[0]
    work_chunk = chunks[1]

    assert rest_chunk.document_id == "session-states"
    assert rest_chunk.metadata["document_title"] == "机械臂会话状态"
    assert rest_chunk.metadata["category"] == "state"
    assert rest_chunk.metadata["source"] == "rosclaw-mini-project"
    assert rest_chunk.metadata["version"] == "1.0"
    assert rest_chunk.metadata["risk_level"] == "high"
    assert rest_chunk.metadata["tags"] == [
        "state",
        "safety",
        "workflow",
    ]

    # 每个 Chunk 应拥有独立的 metadata 字典。
    assert rest_chunk.metadata is not work_chunk.metadata


def test_empty_document_produces_no_chunks() -> None:
    document = make_document("")

    chunks = chunk_markdown_document(document)

    assert chunks == []