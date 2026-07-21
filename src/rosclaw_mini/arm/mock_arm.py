from threading import Event, Lock

from rosclaw_mini.arm.base import ArmAdapter


class MockArmAdapter(ArmAdapter):
    """
    模拟机械臂 Adapter。

    它实现 ArmAdapter 规定的统一原子操作，
    但不会调用真实驱动，也不会控制真实机械臂。

    它只通过修改内部状态，模拟机械臂已经执行了操作。
    """

    def __init__(self, move_duration_seconds: float = 0.0):
        self._is_connected: bool = False  # 模拟机械臂连接状态。
        # 模拟夹爪 TCP 在机械臂底座坐标系中的绝对位置。
        self.position: tuple[float, float, float] | None = None

        # 模拟夹爪状态：
        # True 表示打开，False 表示关闭，None 表示尚未操作。
        self.gripper_is_open: bool | None = None

        # 模拟机械臂是否执行了停止命令。
        self.is_stopped: bool = False
        self.torque_enabled: bool = False

        # stop() 通过这个 Event 唤醒正在等待的 move_to()。
        self._stop_event = Event()

        # 表示 move_to() 已经真正进入运动阶段，供线程测试同步使用。
        self._move_started_event = Event()

        # 保护 _is_moving，避免 move_to() 与 stop() 同时读写状态。
        self._state_lock = Lock()
        self._is_moving = False

        self.move_duration_seconds = move_duration_seconds

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

    def wait_until_moving(self, timeout: float) -> bool:
        """
        等待 move_to() 真正开始执行。

        主要用于线程测试，避免测试结果依赖后台线程的调度速度。
        """
        return self._move_started_event.wait(timeout=timeout)

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
    ) -> None:
        with self._state_lock:
            self._is_moving = True
            self._move_started_event.set()

        try:
            # 最多等待指定时间；运动中调用 stop() 会提前结束等待。
            stop_requested = self._stop_event.wait(
                timeout=self.move_duration_seconds
            )

            if stop_requested:
                self.is_stopped = True
                raise RuntimeError("移动操作被停止。")

            # 只有正常完成后，才记录已经到达目标位置。
            self.position = (x, y, z)
            self.is_stopped = False
        finally:
            # 正常完成和异常中断都必须清理本次运动的线程状态。
            self._stop_event.clear()

            with self._state_lock:
                self._is_moving = False
                self._move_started_event.clear()

    def open_gripper(self) -> None:
        # 模拟打开夹爪。
        self.gripper_is_open = True

    def close_gripper(self) -> None:
        # 模拟关闭夹爪。
        self.gripper_is_open = False

    def stop(self) -> None:
        # 无论是否正在运动，都记录已经收到停止命令。
        self.is_stopped = True

        # 只有确实正在运动时才设置 Event，避免空闲 stop 误伤下一次运动。
        with self._state_lock:
            if self._is_moving:
                self._stop_event.set()

    def disable_torque(self, *, emergency: bool = False) -> None:
        # 模拟关闭全部关节力矩。
        self.torque_enabled = False
