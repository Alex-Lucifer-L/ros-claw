from rosclaw_mini.arm.base import ArmAdapter
from rosclaw_mini.command_schema.commands import Command, ExecutionResult

class ArmHandler(ArmAdapter):
    """
    机械臂处理器类（继承自ArmAdapter）：
    这个类实现了机械臂的基本原子操作，包括移动、打开夹爪、关闭夹爪和停止动作。
    """

    def __init__(self,adapter: ArmAdapter):
        self.adapter = adapter

    def move_arm(self, command: Command) -> ExecutionResult:
        """
        执行机械臂移动命令
        """
        x=command.params["x"]
        y=command.params["y"]
        z=command.params["z"]

        self.adapter.move_arm(x, y, z)
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message=f"机械臂已移动到位置: {x}, {y}, {z}",
        )
    
    def open_gripper(self, command: Command) -> ExecutionResult:
        self.adapter.open_gripper()

        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message="机械臂夹爪已打开",
        )