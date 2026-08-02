from rosclaw_mini.llm.fake_client import FakeLLMClient


def test_fake_llm_client_returns_configured_response() -> None:
    expected_response = (
        '{"skill_name": "open_gripper", "params": {}}'
    )

    client = FakeLLMClient(response=expected_response)

    actual_response = client.generate("请打开夹爪")

    assert actual_response == expected_response