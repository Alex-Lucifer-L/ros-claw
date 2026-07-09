from dataclasses import dataclass

@dataclass
class Command:
    """
    机械臂命令的结构体：
    这个结构体定义了机械臂命令的基本信息，包括命令ID、技能名称、参数和来源
    命令ID： 用于唯一标识每个命令，方便追踪和调试。
    技能名称： 指定命令对应的技能或功能的名称。
    参数： 一个字典，包含命令执行所需的参数和其对应的值。
    来源： 指定命令的来源，可能的值为：'user'、'system'、'external'等，用于区分
    
    """
    command_id: str
    skill_name: str
    params: dict
    source: str
    
@dataclass
class SafetyResult:
    """
    安全结果的结构体：
    这个结构体定义了机械臂命令的安全检查结果，包括命令ID、是否安全、风险级别    
    命令ID： 用于唯一标识每个命令，方便追踪和调试。 
    是否安全： 一个布尔值，指示命令是否安全执行。
    风险级别： 一个字符串，表示命执行前的风险级别，可能的值为：'low'、'medium'、'high'等
    原因： 一个字符串，包含命令执行过程中出现的风险原因

    """

    command_id: str
    is_safe: bool
    risk_level: str
    reason: str

@dataclass
class SkillInfo:
    """
    技能信息结构体：
    这个结构体定义了技能的基本信息，包括技能名称、描述、风险级别和是否启用
    技能名称： 用于唯一标识技能，方便追踪和调试。
    描述： 技能的详细描述，解释技能的用途和作用范围
    风险级别： 技能的风险级别，可能的值为：'low'、'medium'、'high'等，用于评估技能的安全性
    是否启用： 一个布尔值，指示技能是否启

    """

    skill_name: str
    description: str
    risk_level: str
    enabled: bool
    
@dataclass
class ExecutionResult:
    """
    执行结果结构体：
    这个结构体定义了机械臂命令的运行结果，包括命令ID、运行状态和运行结果
    命令ID： 用于唯一标识每个命令，方便追踪和调试。 
    技能名称： 技能名称，用于标识命令所使用的技能。
    运行状态： 一个字符串，表示命令的运行状态，可能的值为：'success'、'failure'等
    运行结果： 一个字符串，包含命令的运行结果，例如执行成功或失败的原因

    """

    command_id: str
    skill_name: str
    success: bool
    message: str    