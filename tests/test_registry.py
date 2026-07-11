from rosclaw_mini.skills.base import ParamSpec, SkillDefinition


def test_skill_definition_holds_parameter_schema():
    skill = SkillDefinition(
        skill_name="test_skill",
        description="test",
        risk_level="low",
        enabled=True,
        params_schema={"value": ParamSpec((int,), min_value=0, max_value=1)},
    )
    assert skill.params_schema["value"].accepted_types == (int,)
    assert skill.allow_extra_params is False
