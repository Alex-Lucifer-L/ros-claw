from rosclaw_mini.arm.base import ArmAdapter
from rosclaw_mini.command_schema.commands import Command, ExecutionResult


class ArmHandlers:
    """
    机械臂 Skill 的执行层。

    映射关系：
        Command / Skill
        → 一个或多个 Adapter 原子操作
        → ExecutionResult

    例如：
        move_arm Command
        → adapter.move_to(x, y, z)
        → ExecutionResult

    未来复杂的 pick Skill 可能是：
        adapter.open_gripper()
        → adapter.move_to(...)
        → adapter.close_gripper()
        → adapter.move_to(...)

    ArmHandlers 不直接调用厂商驱动。
    厂商驱动的差异由具体 Adapter 负责处理。
    """

    def __init__(self, adapter: ArmAdapter):
        # 保存当前 Handler 使用的机械臂 Adapter。
        # 这里既可以传入 MockArmAdapter，也可以传入真实机械臂 Adapter。
        self.adapter = adapter

    def move_arm(self, command: Command) -> ExecutionResult:
        """
        将 move_arm Command 映射为夹爪 TCP 的 move_to() 原子操作。
        """

        # 从系统 Command 中取出 Skill 参数。
        x = command.params["x"]
        y = command.params["y"]
        z = command.params["z"]

        # 调用统一硬件接口。
        # 具体是模拟执行还是真实执行，由传入的 Adapter 决定。
        self.adapter.move_to(x, y, z)

        # 将执行完成的信息转换成系统统一的执行结果。
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message=f"夹爪 TCP 已移动到位置: {x}, {y}, {z}",
        )

    def open_gripper(self, command: Command) -> ExecutionResult:
        """
        将 open_gripper Command 映射为
        adapter.open_gripper() 原子操作。
        """

        self.adapter.open_gripper()

        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message="机械臂夹爪已打开",
        )

    def close_gripper(self, command: Command) -> ExecutionResult:
        """
        将 close_gripper Command 映射为
        adapter.close_gripper() 原子操作。
        """

        self.adapter.close_gripper()

        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message="机械臂夹爪已关闭",
        )

    def stop(self, command: Command) -> ExecutionResult:
        """
        将 stop Command 映射为 adapter.stop() 原子操作。
        """

        self.adapter.stop()

        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message="机械臂已停止所有动作",
        )

    def disable_torque(self, command: Command) -> ExecutionResult:
        """将 disable_torque Command 映射为力矩关闭原子操作。"""

        self.adapter.disable_torque()

        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message="机械臂力矩已关闭",
        )
