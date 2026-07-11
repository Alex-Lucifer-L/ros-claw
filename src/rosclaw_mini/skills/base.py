###此文件定义了技能参数规格和技能定义的结构体，用于描述技能的基本信息和参数要求。

from dataclasses import dataclass
 
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
    required: bool
    min_value: float | None = None
    max_value: float | None = None


@dataclass
class SkillDefinition:
    """
    技能定义的结构体：
    这个结构体定义了技能的基本信息，包括技能名称、描述、参数规格和风险级别
    技能名称： 用于唯一标识技能，方便追踪和调试。
    描述： 技能的详细描述，解释技能的用途和作用范围
    参数规格： 一个字典，包含技能参数的规格信息，键为参数名称，值为 ParamSpec 对象
    e.g. {"param1": ParamSpec(...), "param2": ParamSpec(...)}
    风险级别： 技能的风险级别，可能的值为：'low'、'medium'、'high'等，用于评估技能的安全性

    """

    skill_name: str
    description: str
    risk_level: str
    enabled: bool
    param_schema: dict[str, ParamSpec]
    allow_extra_params: bool = False

