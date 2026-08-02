import pytest

from rosclaw_mini.arm.mock_arm import MockArmAdapter


def test_mock_adapter_connect_and_disconnect():
    adapter = MockArmAdapter()

    assert adapter.is_connected is False

    adapter.connect()
    assert adapter.is_connected is True

    adapter.disconnect()
    assert adapter.is_connected is False


def test_mock_adapter_move_to():
    adapter = MockArmAdapter()

    adapter.move_to(0.5, 0.4, 0.3)

    assert adapter.position == (0.5, 0.4, 0.3)
    assert adapter.is_stopped is False


def test_mock_adapter_reads_saved_tcp_position():
    adapter = MockArmAdapter()

    with pytest.raises(RuntimeError, match="尚无当前 TCP"):
        adapter.read_tcp_position()

    adapter.move_to(0.35, -0.01, 0.24)

    assert adapter.read_tcp_position() == (0.35, -0.01, 0.24)


def test_mock_adapter_gripper_operations():
    adapter = MockArmAdapter()

    assert adapter.gripper_is_open is None

    adapter.open_gripper()
    assert adapter.gripper_is_open is True

    adapter.close_gripper()
    assert adapter.gripper_is_open is False


def test_mock_adapter_stop_and_resume_movement():
    adapter = MockArmAdapter()

    adapter.stop()
    assert adapter.is_stopped is True

    adapter.move_to(0.1, 0.2, 0.3)
    assert adapter.is_stopped is False


def test_mock_adapter_disables_torque():
    adapter = MockArmAdapter()
    adapter.connect()

    adapter.disable_torque()

    assert adapter.torque_enabled is False
