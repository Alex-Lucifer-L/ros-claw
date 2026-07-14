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

class MockArmAdapter(ArmAdapter):
    """
    模拟机械臂 Adapter。

    它实现 ArmAdapter 规定的统一原子操作，
    但不会调用真实驱动，也不会控制真实机械臂。

    它只通过修改内部状态，模拟机械臂已经执行了操作。
    """

    def __init__(self):
        # 模拟机械臂当前所在位置。
        self.position: tuple[float, float, float] | None = None

        # 模拟夹爪状态：
        # True 表示打开，False 表示关闭，None 表示尚未操作。
        self.gripper_is_open: bool | None = None

        # 模拟机械臂是否执行了停止命令。
        self.is_stopped: bool = False

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
    ) -> None:
        # 模拟机械臂移动：不控制硬件，只记录新的目标位置。
        self.position = (x, y, z)
        self.is_stopped = False

    def open_gripper(self) -> None:
        # 模拟打开夹爪。
        self.gripper_is_open = True

    def close_gripper(self) -> None:
        # 模拟关闭夹爪。
        self.gripper_is_open = False

    def stop(self) -> None:
        # 模拟停止机械臂。
        self.is_stopped = True