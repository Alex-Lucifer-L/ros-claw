from __future__ import annotations

import json

import pytest

from rosclaw_mini.vision.exceptions import (
    SceneObservationValidationError,
    VLMResponseParseError,
)
from rosclaw_mini.vision.parser import SceneObservationParser


def valid_payload():
    return {
        "scene_description": "桌面上有一个红色方块。",
        "objects": [
            {
                "name": "red block",
                "category": "block",
                "color": "red",
                "location_in_image": "lower_left",
                "confidence": 0.88,
                "attributes": {"shape": "cube"},
                "bounding_box": [0.1, 0.4, 0.3, 0.8],
            }
        ],
        "warnings": ["轻微遮挡"],
    }


def parse(payload):
    return SceneObservationParser().parse(
        json.dumps(payload), source="camera:0", model="fake-vl"
    )


def test_parser_builds_scene_observation_from_valid_json():
    observation = parse(valid_payload())
    assert observation.scene_description == "桌面上有一个红色方块。"
    assert observation.source == "camera:0"
    assert observation.model == "fake-vl"
    assert observation.objects[0].bounding_box == (0.1, 0.4, 0.3, 0.8)
    assert observation.warnings == ("轻微遮挡",)


def test_parser_strips_markdown_json_fence():
    raw = "```json\n" + json.dumps(valid_payload()) + "\n```"
    observation = SceneObservationParser().parse(
        raw, source="image:test.jpg", model="fake-vl"
    )
    assert observation.objects[0].name == "red block"


def test_parser_rejects_invalid_json():
    with pytest.raises(VLMResponseParseError, match="不是有效 JSON"):
        SceneObservationParser().parse(
            "not json", source="camera:0", model="fake-vl"
        )


def test_parser_rejects_objects_that_are_not_a_list():
    payload = valid_payload()
    payload["objects"] = {}
    with pytest.raises(SceneObservationValidationError, match="objects 必须是列表"):
        parse(payload)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("location_in_image", "near_robot", "location_in_image 非法"),
        ("confidence", 1.1, "confidence 超出"),
        ("bounding_box", [0.1, 0.2, 1.2, 0.8], "bounding_box 必须位于"),
        ("bounding_box", [0.8, 0.2, 0.1, 0.7], "最小值不能大于最大值"),
    ],
)
def test_parser_rejects_invalid_object_fields(field, value, message):
    payload = valid_payload()
    payload["objects"][0][field] = value
    with pytest.raises(SceneObservationValidationError, match=message):
        parse(payload)


@pytest.mark.parametrize("coordinate_key", ["x", "robot_y", "tcp_position"])
def test_parser_rejects_robot_coordinate_fields_at_any_depth(coordinate_key):
    payload = valid_payload()
    payload["objects"][0]["attributes"][coordinate_key] = 0.3
    with pytest.raises(SceneObservationValidationError, match="禁止包含机械臂"):
        parse(payload)


def test_parser_fills_optional_object_defaults():
    observation = parse(
        {
            "scene_description": "空桌面。",
            "objects": [{"name": "table"}],
        }
    )
    item = observation.objects[0]
    assert item.location_in_image == "unknown"
    assert item.category is None
    assert item.color is None
    assert item.confidence is None
    assert item.attributes == {}
    assert item.bounding_box is None
    assert observation.warnings == ()

