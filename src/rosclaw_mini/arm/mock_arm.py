from rosclaw_mini.arm.base import ArmAdapter
from threading import Event


class MockArmAdapter(ArmAdapter):
    """
    模拟机械臂 Adapter。

    它实现 ArmAdapter 规定的统一原子操作，
    但不会调用真实驱动，也不会控制真实机械臂。

    它只通过修改内部状态，模拟机械臂已经执行了操作。
    """

    def __init__(self):

        self._is_connected: bool = False  # 模拟机械臂连接状态。
        # 模拟夹爪 TCP 在机械臂底座坐标系中的绝对位置。
        self.position: tuple[float, float, float] | None = None

        # 模拟夹爪状态：
        # True 表示打开，False 表示关闭，None 表示尚未操作。
        self.gripper_is_open: bool | None = None

        # 模拟机械臂是否执行了停止命令。
        self.is_stopped: bool = False
        self.torque_enabled: bool = False
        self._stop_event: Event = Event()  # 用于模拟停止命令的事件标志。


    @property
    def is_connected(self) -> bool:
        # 返回模拟的机械臂连接状态。
        return self._is_connected


    def connect(self) -> None:
        # 模拟连接机械臂。
        self._is_connected = True
        self.torque_enabled = True

    def disconnect(self) -> None:
        # 模拟断开机械臂连接。
        self._is_connected = False
        self.torque_enabled = False

    

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
    ) -> None:
        # 模拟 TCP 移动：不控制硬件，只记录新的目标位置。
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

    def disable_torque(self, *, emergency: bool = False) -> None:
        # 模拟关闭全部关节力矩。
        self.torque_enabled = False
