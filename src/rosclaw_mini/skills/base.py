###此文件定义了技能参数规格和技能定义的结构体，用于描述技能的基本信息和参数要求。
from collections.abc import Callable
from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from dataclasses import dataclass
 
SkillHandler=Callable[[Command], ExecutionResult]
#定义技能处理函数类型，接受一个 Command 对象作为输入，返回一个 ExecutionResult 对象作为输出。

@dataclass
class ParamSpec:
    """
    参数规格的结构体：
    这个结构体定义了技能参数的规格信息，包括参数名称、接受的类型、是否必需、最小值和最大值
    参数名称： 用于唯一标识参数，方便追踪和调试。
    接受的类型： 一个元组，包含参数可以接受的类型，用于验证参数的类型是否正确
    是否必需： 一个布尔值，指示参数是否为必需参数，如果为 True，则在技能执行时必须提供该参数
    最小值： 一个浮点数，表示参数的最小值，如果参数值小于该值，则视为无效参数
    最大值： 一个浮点数，表示参数的最大值，如果参数值大于该值，则视为无效参数

    """

    accepted_types: tuple[type, ...]
    required: bool=True
    min_value: float | None = None
    max_value: float | None = None
    min_inclusive: bool = True
    max_inclusive: bool = True


@dataclass
class SkillDefinition:
    """
    技能定义的结构体：
    这个结构体定义了技能的基本信息和参数要求，包括技能名称、描述、风险等级、是否启用、参数规格和是否允许额外参数
    技能名称： 用于唯一标识技能，方便追踪和调试。
    描述： 一个字符串，描述技能的功能和用途，帮助用户理解技能的作用。
    风险等级： 一个字符串，表示技能的风险等级，用于评估技能的安全性和潜在风险。
    是否启用： 一个布尔值，指示技能是否启用，如果为 False，则技能将被禁用，无法执行。
    参数规格： 一个字典，键为参数名称，值为 ParamSpec 对象，定义了技能参数的规格信息，包括参数名称、接受的类型、是否必需、最小值和最大值。
    e.g. {"param1": ParamSpec(...), "param2": ParamSpec(...)}
    是否允许额外参数： 一个布尔值，指示技能是否允许额外的参数，如果为 True，则在技能执行时可以提供未在参数规格中定义的额外参数；如果为 False，则在技能执行时只能提供在参数规格中定义

    """

    skill_name: str
    description: str
    risk_level: str
    enabled: bool
    params_schema: dict[str, ParamSpec]
    handler: SkillHandler
    allow_extra_params: bool = False
    

