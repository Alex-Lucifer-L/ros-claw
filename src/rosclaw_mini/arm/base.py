from abc import ABC, abstractmethod


class ArmAdapter(ABC):
    """
    机械臂适配器接口类：
    该类定义了机械臂的基本操作接口，包括移动、打开夹爪、关闭夹爪和停止动作。所有具体的机械臂适配器类都应该继承自该接口，并实现这些方法，以确保与机械臂的交互一致性和可扩展性。
    具体直观通俗理解为：一个定义了机械臂操作规范的模板，任何实际的机械臂实现都必须按照这个模板来提供相应的功能。
    """

    @abstractmethod
    def move_to(
        self,
        x: float,
        y: float,
        z: float,
    ) -> None:
        pass

    @abstractmethod
    def open_gripper(self) -> None:
        pass

    @abstractmethod
    def close_gripper(self) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass