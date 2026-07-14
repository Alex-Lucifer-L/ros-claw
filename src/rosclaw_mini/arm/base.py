from abc import ABC, abstractmethod


class ArmAdapter(ABC):
    """
    机械臂硬件适配接口。

    映射关系：
        不同厂商的驱动函数
        → ArmAdapter 统一原子操作

    例如，不同驱动可能分别使用：
        driver.send_coords(...)
        driver.set_position(...)
        ROS publisher.publish(...)

    经过对应的 Adapter 包装后，上层统一调用：
        adapter.move_to(...)
        adapter.open_gripper()
        adapter.close_gripper()
        adapter.stop()

    ArmAdapter 只负责统一硬件操作，不负责：
        1. 解析 Command
        2. 参数验证和安全检查
        3. 编排 pick、place 等复杂 Skill
        4. 生成 ExecutionResult
    """

    @abstractmethod
    def move_to(
        self,
        x: float,
        y: float,
        z: float,
    ) -> None:
        """
        将不同厂商的机械臂移动函数，
        统一映射为 move_to(x, y, z)。
        """
        pass

    @abstractmethod
    def open_gripper(self) -> None:
        """
        将不同厂商的夹爪打开函数，
        统一映射为 open_gripper()。
        """
        pass

    @abstractmethod
    def close_gripper(self) -> None:
        """
        将不同厂商的夹爪关闭函数，
        统一映射为 close_gripper()。
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """
        将不同厂商的停止函数，
        统一映射为 stop()。
        """
        pass