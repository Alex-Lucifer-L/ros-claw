from rosclaw_mini.arm.mock_arm import MockArmAdapter


def test_mock_adapter_move_to():
    adapter = MockArmAdapter()

    adapter.move_to(0.5, 0.4, 0.3)

    assert adapter.position == (0.5, 0.4, 0.3)
    assert adapter.is_stopped is False