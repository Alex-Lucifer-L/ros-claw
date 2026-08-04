###此文件定义了知识文档、知识文档分块和检索到的知识文档分块的数据结构。用作为后续的知识检索和处理提供基础数据结构。
from pathlib import Path
from typing import Any

import yaml

from rosclaw_mini.rag.document import KnowledgeDocument


REQUIRED_FIELDS = {
    "document_id",
    "title",
    "category",
    "source",
    "version",
    "risk_level",
}


def load_markdown_document(path: str | Path) -> KnowledgeDocument:
    """
    从 Markdown 文件加载知识文档。
    具体来说，Markdown 文件应包含 YAML Front Matter，其中包含文档的元数据。
    必须包含以下字段：
    - document_id
    - title
    - category
    - source
    - version
    - risk_level    
    加载流程是：
    1. 读取文件内容。
    2. 提取 YAML Front Matter。
    3. 验证必填字段是否存在。
    4. 将剩余的 YAML 字段作为 metadata。
    5. 返回 KnowledgeDocument 实例。
    """
    file_path = Path(path)

    raw_text = file_path.read_text(encoding="utf-8")
    lines = raw_text.splitlines()

    if not lines or lines[0].strip() != "---":
        raise ValueError(
            f"知识文档缺少 YAML Front Matter：{file_path}"
        )

    end_index = None

    for index, line in enumerate(lines[1:], start=1):
        if line.strip() == "---":
            end_index = index
            break

    if end_index is None:
        raise ValueError(
            f"知识文档的 YAML Front Matter 没有结束标记：{file_path}"
        )

    front_matter_text = "\n".join(lines[1:end_index])
    content = "\n".join(lines[end_index + 1:]).strip()

    front_matter: Any = yaml.safe_load(front_matter_text)

    if not isinstance(front_matter, dict):
        raise ValueError(
            f"知识文档的 YAML Front Matter 必须是字典结构：{file_path}"
        )

    missing_fields = REQUIRED_FIELDS - front_matter.keys()

    if missing_fields:
        missing_text = ", ".join(sorted(missing_fields))
        raise ValueError(
            f"知识文档缺少必填字段：{missing_text}"
        )

    metadata = {
        key: value
        for key, value in front_matter.items()
        if key not in REQUIRED_FIELDS
    }

    return KnowledgeDocument(
        document_id=str(front_matter["document_id"]),
        title=str(front_matter["title"]),
        category=str(front_matter["category"]),
        content=content,
        source=str(front_matter["source"]),
        version=str(front_matter["version"]),
        risk_level=str(front_matter["risk_level"]),
        metadata=metadata,
    )