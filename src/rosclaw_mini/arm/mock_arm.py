from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.arm.base import ArmAdapter


def move_arm(command: Command) -> ExecutionResult:
    """
    执行机械臂移动命令
    """
    # 这里可以添加实际的机械臂控制逻辑
    # 例如，调用机械臂的API来移动到指定位置
    # 目前仅返回一个模拟的执行结果
    return ExecutionResult(
        command_id=command.command_id,
        skill_name=command.skill_name,
        success=True,
        message=f"机械臂已移动到位置: {command.params}"
    )

def open_gripper(command: Command) -> ExecutionResult:
    """
    执行机械臂夹爪打开命令
    """
    # 这里可以添加实际的机械臂控制逻辑
    # 例如，调用机械臂的API来打开夹爪
    # 目前仅返回一个模拟的执行结果
    return ExecutionResult(
        command_id=command.command_id,
        skill_name=command.skill_name,
        success=True,
        message="机械臂夹爪已打开"
    )

def close_gripper(command: Command) -> ExecutionResult:
    """
    执行机械臂夹爪关闭命令
    """
    # 这里可以添加实际的机械臂控制逻辑
    # 例如，调用机械臂的API来关闭夹爪
    # 目前仅返回一个模拟的执行结果
    return ExecutionResult(
        command_id=command.command_id,
        skill_name=command.skill_name,
        success=True,
        message="机械臂夹爪已关闭"
    )

def stop(command: Command) -> ExecutionResult:
    """
    执行机械臂停止命令
    """
    # 这里可以添加实际的机械臂控制逻辑
    # 例如，调用机械臂的API来停止所有动作
    # 目前仅返回一个模拟的执行结果
    return ExecutionResult(
        command_id=command.command_id,
        skill_name=command.skill_name,
        success=True,
        message="机械臂已停止所有动作"
    )

class MockArmArmAdapter(ArmAdapter):
    """
    模拟机械臂适配器，用于测试和开发阶段。
    它实现了 ArmAdapter 接口，但不与实际的机械臂硬件交互。
    该类用于在没有实际机械臂的情况下进行功能验证和测试。
    通过使用 MockArmAdapter，可以模拟机械臂的行为，记录命令的执行情况，并提供可预测的响应，从而帮助开发人员在软件层面上进行调试和验证，而无需依赖实际的机械臂硬件。
    这对于开发和测试阶段非常有用，尤其是在硬件不可用或不便使用的情况下。
    该类的实现方法包括移动机械臂、打开和关闭夹爪，以及停止机械臂动作。每个方法都会记录相应的状态，以便在测试
    """

    def __init__(self):
        self.position:tuple[float,float,float]|None = None
        self.gripper_is_open:bool | None = None
        self.is_stopped:bool | None = None

    def move_to(self, x: float, y: float, z: float) -> None:
        self.position = (x, y, z)
        self.is_stopped = False

    def open_gripper(self) -> None:
        self.gripper_is_open = True
        self.is_stopped = False

    def close_gripper(self) -> None:
        self.gripper_is_open = False
        self.is_stopped = False

    def stop(self) -> None:
        self.is_stopped = True      