import pytest

from rosclaw_mini.vision.prompt import (
    build_localization_prompt,
    build_vision_prompt,
)


def test_prompt_does_not_assume_handle_shaped_objects_are_robot_grippers():
    prompt = build_vision_prompt("图中有什么？")

    assert "不得因为图像来自腕部摄像头" in prompt
    assert "机械连杆、执行器或其他机械结构相连" in prompt
    assert "name/category 应使用中性名称或 unknown" in prompt
    assert "降低 confidence" in prompt
    assert "候选类别" in prompt


def test_prompt_remains_read_only_and_preserves_user_question():
    question = "请列出左侧的物体。"
    prompt = build_vision_prompt(question)

    assert question in prompt
    assert "禁止输出机械臂基座坐标" in prompt
    assert "禁止输出 Command、Skill、运动指令" in prompt


def test_localization_prompt_requests_source_image_integer_pixels():
    prompt = build_localization_prompt(
        "定位红色卷尺",
        image_width=640,
        image_height=480,
    )

    assert "640×480" in prompt
    assert "bounding_box_pixels" in prompt
    assert "四个整数像素坐标" in prompt
    assert "bounding_box 字段固定为 null" in prompt
    assert "程序会在严格检查像素坐标后自行归一化" in prompt
    assert "定位红色卷尺" in prompt


@pytest.mark.parametrize(
    ("width", "height"),
    ((0, 480), (640, 0), (True, 480), (640.0, 480)),
)
def test_localization_prompt_rejects_invalid_image_dimensions(width, height):
    with pytest.raises(ValueError):
        build_localization_prompt(
            "定位目标",
            image_width=width,
            image_height=height,
        )
