from rosclaw_mini.arm.base import ArmAdapter

class SO100PlusAdapter(ArmAdapter):
    """
    SO100PlusAdapter 是针对 SO100 Plus 机械臂的适配器类。

    它实现了 ArmAdapter 规定的统一原子操作，
    并通过调用 SO100 Plus 的驱动接口来控制机械臂。
    """

    def __init__(self):
        # 初始化 SO100 Plus 机械臂的驱动接口。
        self._is_connected: bool = False  # 机械臂连接状态。
        self.position: tuple[float, float, float] | None = None  # 当前所在位置。
        self.gripper_is_open: bool | None = None  # 夹爪状态。
        self.is_stopped: bool = False  # 是否执行了停止命令。

    @property
    def is_connected(self) -> bool:
        # 返回机械臂连接状态。
        return self._is_connected

    def connect(self) -> None:
        # 调用 SO100 Plus 驱动接口连接机械臂。
        if not self._is_connected:
            # 这里可以添加实际的连接逻辑，例如调用驱动库的 connect 方法。
            self._is_connected = True

    def disconnect(self) -> None:
        # 调用 SO100 Plus 驱动接口断开机械臂连接。
        if self._is_connected:
            # 这里可以添加实际的断开连接逻辑，例如调用驱动库的 disconnect 方法。
            self._is_connected = False

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
    ) -> None:
        # 调用 SO100 Plus 驱动接口移动机械臂到指定位置。
        raise NotImplementedError("SO100 Plus 驱动接口的移动功能尚未实现。")

    def open_gripper(self) -> None:
        # 调用 SO100 Plus 驱动接口打开夹爪。
        raise NotImplementedError("SO100 Plus 驱动接口的打开夹爪功能尚未实现。")

    def close_gripper(self) -> None:
        # 调用 SO100 Plus 驱动接口关闭夹爪。
        raise NotImplementedError("SO100 Plus 驱动接口的关闭夹爪功能尚未实现。")
    
    def stop(self) -> None:
        # 调用 SO100 Plus 驱动接口停止机械臂的运动。
        raise NotImplementedError("SO100 Plus 驱动接口的停止功能尚未实现。")