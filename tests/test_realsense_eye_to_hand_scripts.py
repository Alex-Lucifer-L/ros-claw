from __future__ import annotations

from pathlib import Path

from rosclaw_mini.vision.eye_to_hand import load_eye_to_hand_dataset
from scripts.record_realsense_eye_to_hand_point import main


def test_record_script_creates_and_appends_hashed_dataset(tmp_path: Path):
    dataset = tmp_path / "point_pairs.json"
    outputs = []
    base_args = [
        "--dataset",
        str(dataset),
        "--serial",
        "serial-1",
        "--width",
        "640",
        "--height",
        "480",
    ]

    first = main(
        [
            *base_args,
            "--camera-point",
            "0.1",
            "0.2",
            "0.5",
            "--base-point",
            "0.3",
            "-0.1",
            "0.2",
        ],
        output_func=outputs.append,
    )
    second = main(
        [
            *base_args,
            "--camera-point",
            "0.2",
            "0.1",
            "0.45",
            "--base-point",
            "0.4",
            "-0.2",
            "0.15",
            "--split",
            "validation",
        ],
        output_func=outputs.append,
    )

    assert first == second == 0
    adapter = load_eye_to_hand_dataset(dataset)
    assert [point.point_id for point in adapter.points] == [
        "point_001",
        "point_002",
    ]
    assert adapter.points[1].split == "validation"
    assert "dataset_sha256=" in outputs[-1]


def test_record_script_rejects_mismatched_device(tmp_path: Path):
    dataset = tmp_path / "point_pairs.json"
    common = [
        "--dataset",
        str(dataset),
        "--width",
        "640",
        "--height",
        "480",
        "--camera-point",
        "0.1",
        "0.2",
        "0.5",
        "--base-point",
        "0.3",
        "-0.1",
        "0.2",
    ]
    assert main([*common, "--serial", "serial-1"]) == 0
    outputs = []
    assert (
        main(
            [*common, "--serial", "serial-2"],
            output_func=outputs.append,
        )
        == 1
    )
    assert "相机序列号不匹配" in outputs[0]
