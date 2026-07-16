# RosClaw Mini

RosClaw Mini 是一个面向机械臂控制的轻量级 Python 原型。项目把用户命令依次交给
结构校验、安全检查、Skill Gateway 和统一 `ArmAdapter`，避免上层逻辑直接调用
电机驱动。

> 默认 `main.py` 仍使用 `MockArmAdapter`，不会自动连接真实机械臂。
> SO-100 Plus 的底层接入和手动真机验证已实现，但尚未提供面向终端用户的
> 真机 Gateway 启动入口。本项目不应用于无人值守或生产环境。

## 当前状态

已实现的命令链路：

```text
Command
→ Skill Registry
→ Enabled Check
→ Validator
→ Safety Checker
→ Gateway
→ ArmHandlers
→ ArmAdapter
→ ExecutionResult
```

真实 SO-100 Plus 的底层映射：

```text
上层统一原子动作
→ SO100PlusAdapter
→ LeRobot ManipulatorRobot
→ FeetechMotorsBus
→ 7 个 STS3215 电机
```

### 命令与安全链路

- `Command`、`SafetyResult`、`ExecutionResult` 数据模型
- `SkillDefinition` 和 `ParamSpec`
- 必需参数、类型、额外参数校验
- 基于 `ParamSpec` 的通用数值边界检查
- Gateway 查找、执行和异常转换
- Mock 默认后端和交互式 JSON 入口

### SO-100 Plus

- 可配置的单臂 `ManipulatorRobot` factory
- 串口、follower 名称和校准路径预检
- 7 个电机的位置和遥测读取
- 夹爪打开/关闭、保持位置和关闭力矩
- 基于 `lerobot_kinematics` 的 FK/IK
- 两根夹指之间的真实 TCP
- 米制底座绝对坐标 `move_to(x, y, z)`
- 20 Hz 余弦缓入缓出流式轨迹
- 关节、TCP、负载、温度和跟踪误差保护
- OpenCV RGB 摄像头配置和图像抓取接口
- 不会由普通 `pytest` 执行的手动真机脚本

真机坐标系、TCP、安全参数、运行命令和验证结果见
[SO-100 Plus 动作与安全文档](docs/arm_actions.md)。

## 内置 Skill

| Skill | 风险等级 | 参数 | Adapter 原子动作 | 说明 |
| --- | --- | --- | --- | --- |
| `move_arm` | `medium` | `x`, `y`, `z` | `move_to(x, y, z)` | 只有传入明确 `WorkspaceLimits` 时才启用 |
| `open_gripper` | `low` | 无 | `open_gripper()` | 打开夹爪 |
| `close_gripper` | `low` | 无 | `close_gripper()` | 关闭夹爪 |
| `stop` | `low` | 无 | `stop()` | 保持当前位置，不关闭力矩 |
| `disable_torque` | `medium` | 无 | `disable_torque()` | 关闭力矩，机械臂可能下落 |

`move_arm` 不再使用写死的 `0 < x,y,z <= 1` 真机范围。
`build_arm_skills()` 会把调用方传入的 `WorkspaceLimits` 写入 `ParamSpec`；
如果没有范围，`move_arm` 失败关闭。`main.py` 中的 `[-1, 1]` 只是 Mock 演示范围。

## 快速开始：Mock 后端

### 1. 进入仓库和环境

```bash
cd rosclaw-mini
conda activate rosclaw-mini-py310
```

### 2. 启动交互入口

```bash
PYTHONPATH=src python -m rosclaw_mini.main
```

输入结构化 JSON：

```json
{"skill_name": "move_arm", "params": {"x": 0.5, "y": 0.4, "z": 0.3}}
```

其他示例：

```json
{"skill_name": "open_gripper", "params": {}}
{"skill_name": "close_gripper", "params": {}}
{"skill_name": "stop", "params": {}}
{"skill_name": "disable_torque", "params": {}}
```

输入 `exit` 退出。该入口不调用 LLM，也不连接真机。

## Python 调用示例

Skill 需要绑定到一个具体 Adapter：

```python
from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.safety.limits import AxisLimits, WorkspaceLimits
from rosclaw_mini.skills.arm_skills import build_arm_skills

adapter = MockArmAdapter()
workspace = WorkspaceLimits(
    x=AxisLimits(-1.0, 1.0),
    y=AxisLimits(-1.0, 1.0),
    z=AxisLimits(-1.0, 1.0),
)
skills = build_arm_skills(adapter, workspace_limits=workspace)

command = Command(
    command_id="cmd-001",
    skill_name="move_arm",
    params={"x": 0.5, "y": 0.4, "z": 0.3},
    source="user",
)

result = run_command(command, skills)
print(result)
```

Gateway 会把 Handler 或 Adapter 抛出的异常转换为 `success=False` 的
`ExecutionResult`。

## 单元测试

```bash
conda activate rosclaw-mini-py310
python -m pytest -q
```

当前结果：

```text
157 passed
```

测试覆盖：

- Command、Skill、Validator、Safety Checker 和 Gateway
- Mock Adapter 及 ArmHandlers
- 工作空间、关节限制和运动学
- SO-100 Plus factory、校准预检和 Adapter
- 夹爪、stop、关力矩、轨迹、遥测和错误处理
- Camera factory、连接顺序和图像形状

所有默认测试使用 FakeRobot/FakeBus/FakeCamera，不会读写真实串口或摄像头。

## 手动真机验证

真机验证位于 `scripts/`，不由 `pytest` 执行。这些脚本需要：

- 显式的串口路径；
- 校准目录和 follower 名称；
- 操作者站在机械臂旁边；
- 清空工作区并准备物理断电；
- 显式 `--acknowledge-...-risk` 参数。

示例：生成局部 +Z 1 cm 真机轨迹并执行：

```bash
PYTHONPATH=src:lerobot-joycon_plus \
python scripts/check_so100_plus_adapter_move_to.py \
  --port /dev/lerobot_right \
  --calibration-dir lerobot-joycon_plus/.cache/calibration/so100_plus \
  --follower-name right \
  --delta-z-cm 1 \
  --runtime-acceleration 35 \
  --single-cartesian-plan \
  --stream-frequency-hz 20 \
  --stream-max-joint-speed-degrees-per-second 20 \
  --acknowledge-move-to-risk
```

> 运行上述命令会启用力矩并移动真实机械臂。复制命令前必须先阅读
> [真机风险、坐标和完整操作方法](docs/arm_actions.md#13-手动真机运行方法)。

## 项目结构

```text
rosclaw-mini/
├── README.md
├── docs/
│   └── arm_actions.md                # SO-100 Plus 坐标、限制和运行文档
├── scripts/
│   ├── check_so100_plus_*.py         # 需要显式确认的手动真机检查
│   └── *mujoco*.py                   # 运动学和 MuJoCo 预览
├── src/rosclaw_mini/
│   ├── main.py                         # 默认 Mock JSON 入口
│   ├── arm/
│   │   ├── base.py                    # 统一 ArmAdapter
│   │   ├── mock_arm.py                # Mock 实现
│   │   ├── so100_plus.py              # 真实 Adapter 与运行配置
│   │   ├── so100_plus_factory.py      # Robot/Camera factory 和校准预检
│   │   └── kinematics.py              # FK/IK 和 TCP
│   ├── skills/                         # Skill 定义和 ArmHandlers
│   ├── safety/                         # 命令、工作空间和关节限制
│   └── gateway/                        # 命令执行编排
├── tests/                               # 默认无硬件测试
└── lerobot-joycon_plus/                 # 独立 LeRobot fork
```

## 当前边界

- 全局 XYZ 工作空间和全部关节实机绝对范围尚未完成认证。
- `move_to()` 保持当前姿态，尚不支持显式 roll/pitch/yaw。
- 没有碰撞模型、视觉避障或独立硬件急停接口。
- 真实摄像头已完成软件接入，但尚未执行单帧抓取。
- `main.py` 尚无显式 SO-100 Plus Gateway 后端选择。
- `configs/*.yaml` 尚未接入主链路。
- Python 包和真机依赖尚未完整声明，当前通过 `PYTHONPATH` 运行。
- LLM、RAG、Web 和 ROS 2 仍未接入当前执行链路。

## 下一步

1. 为 `main.py` 增加显式 `mock/so100_plus` 后端选择，保持 Mock 默认。
2. 把已确认的实机工作空间接入 Gateway/Skill `ParamSpec`。
3. 声明可复现的真机运动学和 OpenCV 依赖。
4. 完成摄像头单帧和“摄像头先连接、机械臂后上力”的真机验证。
5. 在真机链路稳定后再扩展 LLM、RAG、Web 或 ROS 2。
