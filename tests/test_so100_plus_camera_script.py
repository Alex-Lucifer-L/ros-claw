from scripts.check_so100_plus_camera import build_parser


def test_camera_check_script_uses_readme_defaults():
    args = build_parser().parse_args(["--device", "/dev/video2"])

    assert args.name == "right"
    assert args.fps == 60
    assert args.width == 640
    assert args.height == 480
    assert args.acknowledge_camera_capture is False
