from abc import ABC, abstractmethod


class ArmAdapter(ABC):
    """
    机械臂统一控制接口。

    MockArm 和真实机械臂 Adapter 都必须实现这些方法。
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