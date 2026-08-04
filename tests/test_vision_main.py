from __future__ import annotations

import json

from rosclaw_mini.main import main, run_vision_command_loop
from rosclaw_mini.vision.schemas import SceneObject, SceneObservation


def observation():
    return SceneObservation(
        observation_id="obs-1",
        timestamp="2026-01-01T00:00:00+00:00",
        scene_description="桌面上有一个红色方块。",
        objects=(
            SceneObject(
                name="red block",
                category="block",
                color="red",
                location_in_image="lower_left",
                confidence=0.88,
            ),
        ),
        warnings=("轻微遮挡",),
        source="camera:0",
        model="fake-vl",
    )


class FakeVisionService:
    def __init__(self):
        self.calls = []

    def observe(self, **kwargs):
        self.calls.append(kwargs)
        return observation()


class InterruptingVisionService:
    def observe(self, **kwargs):
        raise KeyboardInterrupt


def sequence_input(values):
    iterator = iter(values)
    return lambda _prompt: next(iterator)


def test_vision_main_builds_config_and_never_builds_arm_runtime():
    outputs = []
    captured = {}
    fake_service = FakeVisionService()

    def vlm_builder(**kwargs):
        captured["vlm"] = kwargs
        return object()

    def service_builder(**kwargs):
        captured["service"] = kwargs
        return fake_service

    def forbidden_runtime(_args):
        raise AssertionError("vision mode must not create ArmRuntime")

    exit_code = main(
        [
            "--input-mode",
            "vision",
            "--backend",
            "so100_plus",
            "--vlm-model",
            "qwen-vl-test",
            "--vision-question",
            "桌面上有什么？",
        ],
        output_func=outputs.append,
        runtime_builder=forbidden_runtime,
        vlm_client_builder=vlm_builder,
        vision_service_builder=service_builder,
        environ={"ROSCLAW_LLM_API_KEY": "placeholder"},
    )

    assert exit_code == 0
    assert captured["vlm"]["model"] == "qwen-vl-test"
    assert captured["vlm"]["api_key"] == "placeholder"
    assert captured["service"]["camera_index"] == 0
    assert fake_service.calls == [
        {
            "question": "桌面上有什么？",
            "image_path": None,
            "save_frame_path": None,
        }
    ]
    assert any("视觉观察完成" in item for item in outputs)


def test_vision_main_model_environment_and_dashscope_key_fallback():
    captured = {}
    fake_service = FakeVisionService()

    def vlm_builder(**kwargs):
        captured.update(kwargs)
        return object()

    code = main(
        ["--input-mode", "vision", "--vision-question", "观察"],
        vlm_client_builder=vlm_builder,
        vision_service_builder=lambda **kwargs: fake_service,
        environ={
            "DASHSCOPE_API_KEY": "placeholder",
            "DASHSCOPE_VL_MODEL": "env-vl-model",
        },
        output_func=lambda _message: None,
    )
    assert code == 0
    assert captured["model"] == "env-vl-model"
    assert captured["api_key"] == "placeholder"


def test_vision_config_error_happens_before_runtime_creation():
    outputs = []

    def forbidden_runtime(_args):
        raise AssertionError("runtime must not be constructed")

    code = main(
        ["--input-mode", "vision", "--vision-question", "观察"],
        runtime_builder=forbidden_runtime,
        environ={},
        output_func=outputs.append,
    )
    assert code == 2
    assert "API Key" in outputs[0]


def test_vision_ctrl_c_exits_without_arm_runtime():
    outputs = []

    def forbidden_runtime(_args):
        raise AssertionError("runtime must not be constructed")

    code = main(
        ["--input-mode", "vision", "--vision-question", "观察"],
        runtime_builder=forbidden_runtime,
        vlm_client_builder=lambda **kwargs: object(),
        vision_service_builder=lambda **kwargs: InterruptingVisionService(),
        environ={"ROSCLAW_LLM_API_KEY": "placeholder"},
        output_func=outputs.append,
    )
    assert code == 130
    assert "未创建机械臂 Runtime" in outputs[-1]


def test_vision_interactive_observe_ask_and_exit():
    service = FakeVisionService()
    outputs = []
    code = run_vision_command_loop(
        service,
        input_func=sequence_input(
            ["observe", "ask 红色方块在哪里", "exit"]
        ),
        output_func=outputs.append,
    )
    assert code == 0
    assert service.calls[0]["question"] is None
    assert service.calls[1]["question"] == "红色方块在哪里"
    assert outputs[-1] == "退出视觉模式。"


def test_vision_text_output_is_human_readable():
    service = FakeVisionService()
    outputs = []
    code = run_vision_command_loop(
        service,
        question="观察",
        output_format="text",
        output_func=outputs.append,
    )
    assert code == 0
    assert "场景：桌面上有一个红色方块。" in outputs[0]
    assert "画面位置=左下" in outputs[0]
    assert "警告：" in outputs[0]


def test_vision_json_output_is_machine_readable():
    service = FakeVisionService()
    outputs = []
    code = run_vision_command_loop(
        service,
        question="观察",
        output_format="json",
        output_func=outputs.append,
    )
    assert code == 0
    payload = json.loads(outputs[0])
    assert payload["observation_id"] == "obs-1"
    assert payload["objects"][0]["location_in_image"] == "lower_left"
