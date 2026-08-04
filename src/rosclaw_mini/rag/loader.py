"""加载带 YAML Front Matter 的项目知识文档。"""

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

DEFAULT_SKIPPED_FILENAMES = frozenset({"README.md"})


def _require_non_empty_text(
    front_matter: dict[str, Any],
    field_name: str,
    file_path: Path,
) -> str:
    value = front_matter[field_name]
    if not isinstance(value, str) or not value.strip():
        raise ValueError(
            f"知识文档字段 {field_name} 必须是非空字符串：{file_path}"
        )
    return value.strip()


def _validate_string_list(
    value: Any,
    field_name: str,
    file_path: Path,
) -> list[str]:
    if not isinstance(value, list) or any(
        not isinstance(item, str) or not item.strip()
        for item in value
    ):
        raise ValueError(
            f"知识文档字段 {field_name} 必须是非空字符串列表：{file_path}"
        )
    return [item.strip() for item in value]


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

    document_id = _require_non_empty_text(
        front_matter, "document_id", file_path
    )
    title = _require_non_empty_text(front_matter, "title", file_path)
    category = _require_non_empty_text(
        front_matter, "category", file_path
    )
    source = _require_non_empty_text(front_matter, "source", file_path)
    version = _require_non_empty_text(front_matter, "version", file_path)
    risk_level = _require_non_empty_text(
        front_matter, "risk_level", file_path
    )

    metadata = {
        key: value
        for key, value in front_matter.items()
        if key not in REQUIRED_FIELDS
    }
    for list_field in ("tags", "source_files"):
        if list_field in metadata:
            metadata[list_field] = _validate_string_list(
                metadata[list_field], list_field, file_path
            )
    metadata["knowledge_path"] = str(file_path)

    return KnowledgeDocument(
        document_id=document_id,
        title=title,
        category=category,
        content=content,
        source=source,
        version=version,
        risk_level=risk_level,
        metadata=metadata,
    )


def load_markdown_documents(
    directory: str | Path,
    *,
    skipped_filenames: frozenset[str] = DEFAULT_SKIPPED_FILENAMES,
) -> list[KnowledgeDocument]:
    """以稳定顺序加载目录中的检索文档，并拒绝重复文档 ID。"""

    directory_path = Path(directory)
    if not directory_path.exists():
        raise FileNotFoundError(f"知识目录不存在：{directory_path}")
    if not directory_path.is_dir():
        raise NotADirectoryError(f"知识目录不是文件夹：{directory_path}")

    paths = sorted(
        (
            path
            for path in directory_path.rglob("*.md")
            if path.name not in skipped_filenames
            and not path.name.startswith((".", "_"))
        ),
        key=lambda path: path.relative_to(directory_path).as_posix(),
    )
    documents: list[KnowledgeDocument] = []
    document_paths: dict[str, Path] = {}
    for path in paths:
        document = load_markdown_document(path)
        previous_path = document_paths.get(document.document_id)
        if previous_path is not None:
            raise ValueError(
                "知识目录包含重复 document_id "
                f"{document.document_id!r}：{previous_path} 与 {path}"
            )
        document_paths[document.document_id] = path
        documents.append(document)

    return documents
