# 工作空间扫描子系统

这个目录集中保存离线工作空间扫描代码和扫描结果的解释方式。它的目标是
让扫描器脱离 `scripts/`，成为以后可以由程序或 LLM 工具调用的稳定模块，
同时明确区分“仿真候选空间”和“真机正式安全空间”。

## 当前目录

```text
src/rosclaw_mini/workspace_scan/
├── __init__.py
├── so100_plus.py
├── README.md
└── SO100_PLUS_MIDDLE_INTERNAL_10MM.md
```

- `so100_plus.py`：完整扫描实现，包括网格生成、IK、关节限制、MuJoCo
  端点/路径碰撞检查、夹爪姿态交集、邻接边检查、NPZ/JSON/PNG 输出。
- `SO100_PLUS_MIDDLE_INTERNAL_10MM.md`：当前选定的不规则空间扫描快照、
  参数、结果和文件校验和。
- `scripts/simulate_so100_plus_rest_workspace.py`：仅保留兼容命令入口，
  旧命令无需修改。正式代码不再反向依赖 `scripts/`。

生成的二进制网格和图片仍放在仓库顶层 `artifacts/`。这是有意的：源代码
与可重复生成的扫描产物分开，避免把大体积 NPZ 当成 Python 包资源。
历史输出字段和文件名中的 `rest` 为兼容旧扫描器而保留；当
`reference_pose=middle_internal` 时，它们实际表示参考姿态，不表示
`follower_rest` 收纳姿态。

## 扫描器实际证明了什么

对每个笛卡尔网格目标，扫描器会：

1. 从选定参考关节姿态求解保持参考 TCP 朝向的 IK；
2. 校验模型关节范围和实测底座范围；
3. 检查目标姿态是否自碰撞、接触支撑平面或使 TCP 低于桌面；
4. 按指定关节步长检查参考姿态到目标解的完整插值路径；
5. 对命令列出的每一种夹爪姿态重复碰撞检查，最终取共同安全交集；
6. 保存所有网格状态、目标关节解、有效 TCP 点、报告和可视化。

因此，一个有效点的准确含义是：在当前模型、固定 TCP 朝向、指定夹爪
姿态、给定网格分辨率和指定参考路径下，它通过了离线仿真检查。

它不能证明：

- 未采样的连续空间也全部安全；
- 任意两个有效点之间都可直接移动；
- 真实机械臂不存在回差、下垂、线缆、负载、温度或装配误差；
- 有效点 XYZ 的独立极值能够组成一个安全长方体。

## 项目已经确定采用不规则空间

运行时现已加载不规则点集，不再把扫描点压缩成一个 XYZ 外包长方体。
`irregular_workspace.py` 会校验固定 SHA-256、参考姿态、网格、有效点、
夹爪交集和关节范围，并开放全部有效节点及“所有必要角点均有效”的连续
单元。AABB 只供 Gateway 粗筛，不能放行点集空洞。

由于扫描验证的是 `middle_internal → 目标`，运行时采用
`当前位置 → middle_internal → 目标` 中心通道。最终 30 Hz waypoint
仍会用当次实际夹爪 qpos 完整 MuJoCo 预检并原样执行。这个方案无需假设
任意两个有效点之间可以直达；以后若要缩短路径，可以再为全点集生成邻接
图，但不得用未经验证的直线路径替代当前中心通道。

旧 `12 × 6 × 12 cm` 真机代表点范围继续作为已验收核心。核心外的不规则
扩展仍需现场分阶段人工验收，不能因为已经接入运行时就描述成全部真机
认证完成。

## 运行方式

旧入口继续可用：

```bash
PYTHONPATH=src:lerobot-joycon_plus \
MPLCONFIGDIR=/tmp/matplotlib-rosclaw \
python scripts/simulate_so100_plus_rest_workspace.py --help
```

独立包入口也可用：

```bash
PYTHONPATH=src:lerobot-joycon_plus \
MPLCONFIGDIR=/tmp/matplotlib-rosclaw \
python -m rosclaw_mini.workspace_scan.so100_plus --help
```

两者调用同一个 `main()`，不会产生两套扫描算法。

## 以后作为 LLM 功能时的边界

本轮没有注册新的 Skill，也没有让 LLM 自动运行长时间扫描。以后接入时，
建议增加一个只执行离线计算的工具层，而不是让模型拼接 shell 命令。工具
输入应使用经过 Validator 校验的结构化字段，例如：

```text
robot_profile
reference_pose
grid_step_mm
path_step_degrees
gripper_driver_degrees
bounds_m
output_name
```

LLM 只负责把自然语言转换成这些字段。工具层负责：固定模型目录和输出根
目录、限制网格规模、拒绝路径穿越、运行扫描、返回报告摘要。扫描功能不能
连接硬件，也不能自动把候选结果升级为正式工作空间。

## 迁移到其他机械臂

算法思路可以迁移，但 `so100_plus.py` 当前仍包含 SO-100 Plus 专属配置。
移植时必须提供并验证：

- MuJoCo 模型及碰撞几何；
- 手臂关节名称、顺序、数量和模型 qpos 映射；
- 正/逆运动学实现和 TCP 偏移；
- 真实驱动范围与模型关节范围的交集；
- 参考关节姿态及固定 TCP 朝向；
- 夹爪反馈到模型 qpos 的正式映射；
- 支撑平面、允许接触和禁止接触规则。

可直接复用的算法主要是中心对齐网格、状态编码、最大有效框统计、邻接边
枚举、报告结构和可视化。以后支持第二种机械臂时，应先抽象
`WorkspaceScanProfile`，让通用扫描引擎依赖 profile；不要在当前文件里继续
堆叠第二套关节名称和特例。

## 无硬件保证

扫描模块没有串口参数，不创建 LeRobot Robot，不访问 `/dev`，也不启用或
关闭电机力矩。它只读取本地模型、候选报告并写入指定输出目录。
