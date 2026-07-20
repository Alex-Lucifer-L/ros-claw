###此文件定义了一个名为 ExecutionController 的类，它负责管理命令的执行。该类使用一个线程来运行命令，以便在后台执行任务，而不会阻塞主线程。初学者可以将其理解为一个“命令执行器”，它接收命令并在单独的线程中运行这些命令，从而实现异步执行。
from collections.abc import Callable
from threading import Thread, Lock

from rosclaw_mini.command_schema.commands import Command, ExecutionResult
CommandRunner = Callable[[Command], ExecutionResult]###表示一个可调用对象（函数或方法），它接受一个 Command 对象并返回一个 ExecutionResult 对象。这个类型别名用于表示执行命令的逻辑。

class ExecutionController:
    """
    这个类负责管理命令的执行。它使用一个线程来运行命令，以便在后台执行任务，而不会阻塞主线程。
    初学者可以将其理解为一个“命令执行器”，它接收命令并在单独的线程中运行这些命令，从而实现异步执行。
    runner: 一个可调用对象（函数或方法），它接受一个 Command 对象并返回一个 ExecutionResult 对象。这个 runner 实际上是执行命令的逻辑。
    worker: 一个线程对象，用于在后台执行命令。初学者可以将其理解为一个“工作线程”，它负责实际运行命令。
    """
    def __init__(self, runner: CommandRunner):
        """
        初始化 ExecutionController。
        runner: 一个可调用对象（函数或方法），它接受一个 Command 对象并返回一个 ExecutionResult 对象。这个 runner 实际上是执行命令的逻辑。
        worker: 一个线程对象，用于在后台执行命令。初学者可以将其理解为一个“工作线程”，它负责实际运行命令。
        running: 一个布尔值，表示是否有命令正在执行。初学者可以将其理解为一个“运行状态”，它告诉我们当前是否有命令在运行。
        locked: 一个线程锁，用于确保在多线程环境中对运行状态的访问是安全的。初学者可以将其理解为一个“锁”，它防止多个线程同时修改运行状态，从而避免潜在的竞争条件。
        last_result: 一个 ExecutionResult 对象，保存上一次命令的执行结果。初学者可以将其理解为一个“结果缓存”，它存储了上一次命令执行的结果，以便后续使用。
        """
        self._runner = runner
        self._worker: Thread | None = None
        self._running: bool = False
        self._lock = Lock()
        self._last_result: ExecutionResult | None = None


    def submit(self, command: Command) -> bool:
        """
        提交一个命令以在后台执行。这个方法会检查当前是否有命令正在执行，如果没有，它会启动一个新的线程来运行命令。  
        返回 True 如果命令成功提交，返回 False 如果当前有命令正在执行。
        """
        if command.skill_name == "stop":
            raise ValueError("stop 命令必须使用 request_stop 方法提交，而不是 submit 方法。")
        with self._lock:
            if self._running:
                return False
            self._running = True
            self._last_result = None
            self._worker = Thread(target=self._run_in_background, args=(command,))
        self._worker.start()
        return True
        
    def request_stop(self,command: Command) -> ExecutionResult:
        """
        请求停止当前正在执行的命令。这个方法会检查当前是否有命令正在执行，如果有，它会尝试停止该命令。  
        返回 True 如果成功请求停止，返回 False 如果当前没有命令正在执行。
        """
        if command.skill_name != "stop":
            raise ValueError("request_stop 方法只能用于 stop 命令。")
        
        return self._runner(command)

    def _run_in_background(self, command: Command) -> None:

        """
        这个方法在后台线程中运行命令。它调用 runner 来执行命令，并在完成后更新运行状态。    

        """
        result: ExecutionResult | None = None
        
        
        try:
            result = self._runner(command)
        except Exception as error:
            result = ExecutionResult(
                command_id=command.command_id,
                skill_name=command.skill_name,
                success=False,
                message=str(error)
            )
        finally:
            with self._lock:
                self._running = False
                self._last_result = result

    def is_running(self) -> bool:
        """
        检查当前是否有命令正在执行。这个方法会返回一个布尔值，表示是否有命令正在运行。
        """
        with self._lock:
            return self._running
    def last_result(self) -> ExecutionResult | None:
        """
        获取上一次命令的执行结果。这个方法会返回一个 ExecutionResult 对象，表示上一次命令的执行结果。
        如果没有命令执行过，则返回 None。
        """
        with self._lock:
            return self._last_result
        
    def wait(self,timeout: float | None = None) -> ExecutionResult | None:
        """
        等待当前正在执行的命令完成。这个方法会阻塞调用线程，直到命令执行完成或超时。
        timeout: 等待的最长时间（以秒为单位）。如果为 None，则无限期等待。
        返回命令的执行结果，如果命令尚未完成，则返回 None。

        """
        with self._lock:
            worker = self._worker
        if worker is None:
            return self._last_result
        worker.join(timeout)
        with self._lock:
            if self._running:
                return None
            return self._last_result