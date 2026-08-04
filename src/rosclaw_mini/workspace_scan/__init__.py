"""离线工作空间扫描和只读运行时网格。

扫描产物只有在生产加载器核对 SHA-256、参考姿态和网格元数据后，
才会被 SO-100 Plus 会话用作不规则目标门禁。
"""


def scan_so100_plus_workspace() -> int:
    """延迟加载 SO-100 Plus CLI，避免导入包时初始化 MuJoCo。"""

    from .so100_plus import main

    return main()


__all__ = ["scan_so100_plus_workspace"]
