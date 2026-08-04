from pathlib import Path

from rosclaw_mini.rag.chunker import chunk_markdown_document
from rosclaw_mini.llm.command_generator import CommandGenerator
from rosclaw_mini.rag.context import RagContextProvider
from rosclaw_mini.rag.loader import load_markdown_document
from rosclaw_mini.rag.loader import load_markdown_documents
from rosclaw_mini.rag.retriever import KeywordRetriever
from rosclaw_mini.runtime import build_mock_runtime


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_session_states_document_can_be_retrieved() -> None:
    document_path = (
        PROJECT_ROOT
        / "knowledge"
        / "session_states.md"
    )

    document = load_markdown_document(document_path)
    chunks = chunk_markdown_document(document)

    retriever = KeywordRetriever(chunks)

    results = retriever.retrieve(
        "REST 状态下为什么不能直接移动？",
        top_k=3,
    )

    assert chunks
    assert results

    best_result = results[0]

    assert best_result.chunk.metadata["section"] == "REST"
    assert "REST" in best_result.chunk.content
    assert best_result.score > 0


def test_real_knowledge_directory_loads_full_pipeline() -> None:
    knowledge_directory = PROJECT_ROOT / "knowledge"
    provider = RagContextProvider.from_directory(
        knowledge_directory,
        top_k=3,
    )

    rest_results = provider.retrieve("REST 状态 unfold_arm")
    stop_results = provider.retrieve("stop 中断 Event Lock 后台线程")
    skill_results = provider.retrieve("move_relative dx dy dz 参数协议")

    assert any(
        result.chunk.document_id == "session-states"
        for result in rest_results
    )
    assert any(
        result.chunk.document_id == "execution-and-stop"
        for result in stop_results
    )
    assert any(
        result.chunk.document_id == "command-and-skill-contracts"
        for result in skill_results
    )
    assert all(
        result.chunk.metadata.get("source_files")
        for result in rest_results + stop_results + skill_results
    )
    assert len(load_markdown_documents(knowledge_directory)) >= 9


def test_rag_prompt_reaches_fake_llm_for_gripper_and_motion() -> None:
    runtime = build_mock_runtime(move_duration_seconds=0.0)
    provider = RagContextProvider.from_directory(
        PROJECT_ROOT / "knowledge",
        top_k=3,
    )
    prompts: list[str] = []
    responses = iter(
        (
            '{"skill_name":"open_gripper","params":{}}',
            '{"skill_name":"move_relative","params":'
            '{"dx":0.0,"dy":0.0,"dz":0.02}}',
        )
    )

    class RecordingClient:
        def generate(self, prompt: str) -> str:
            prompts.append(prompt)
            return next(responses)

    generator = CommandGenerator(
        client=RecordingClient(),
        skills=runtime.skills,
        context_provider=provider,
        runtime_state_provider=lambda: (
            "backend=mock; session_state=NOT_APPLICABLE"
        ),
    )

    gripper = generator.generate("请打开夹爪")
    motion = generator.generate("请向上移动2厘米")

    assert gripper.skill_name == "open_gripper"
    assert motion.skill_name == "move_relative"
    assert motion.params == {"dx": 0.0, "dy": 0.0, "dz": 0.02}
    assert all("[PROJECT_KNOWLEDGE]" in prompt for prompt in prompts)
    assert all("[SOURCE:" in prompt for prompt in prompts)
    assert "请打开夹爪" in prompts[0]
    assert "请向上移动2厘米" in prompts[1]
    assert "[RUNTIME_STATE]" in prompts[1]
    runtime.shutdown()
