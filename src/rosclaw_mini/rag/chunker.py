"""按 Markdown 二级标题切分项目知识。"""

from rosclaw_mini.rag.document import (
    KnowledgeChunk,
    KnowledgeDocument,
)


def chunk_markdown_document(
    document: KnowledgeDocument,
) -> list[KnowledgeChunk]:
    """
    将 Markdown 格式的知识文档分块为多个 KnowledgeChunk。
    分块的依据是 Markdown 中的二级标题（##）。
    每个分块的 metadata 中会包含原始文档的元数据以及当前分块的 section 信息。
    具体来说，分块流程是：
    1. 遍历文档内容的每一行。
    2. 当遇到二级标题时，保存当前分块并开始新的分块。
    3. 将当前行添加到当前分块的内容中。
    4. 在遍历结束后，保存最后一个分块。
    """
    chunks: list[KnowledgeChunk] = []

    current_lines: list[str] = []
    current_section = document.title

    def save_current_chunk() -> None:
        content = "\n".join(current_lines).strip()

        if not content:
            return

        chunk_index = len(chunks)

        metadata = dict(document.metadata)
        metadata.update(
            {
                "document_title": document.title,
                "category": document.category,
                "source": document.source,
                "version": document.version,
                "risk_level": document.risk_level,
                "section": current_section,
                "document_id": document.document_id,
            }
        )

        chunks.append(
            KnowledgeChunk(
                chunk_id=f"{document.document_id}-{chunk_index}",
                document_id=document.document_id,
                chunk_index=chunk_index,
                content=content,
                metadata=metadata,
            )
        )

    for line in document.content.splitlines():
        if line.startswith("## "):
            save_current_chunk()

            current_lines = [line]
            current_section = line[3:].strip()
        else:
            current_lines.append(line)

    save_current_chunk()

    return chunks
