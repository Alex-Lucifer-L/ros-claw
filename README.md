# RosClaw Mini

RosClaw Mini 是一个面向机械臂控制场景的轻量级 Python 原型。项目当前聚焦于一条可验证的结构化命令执行链路：先从技能注册表中查找技能，完成启用状态、参数结构和安全边界检查，再调用该技能注册的处理函数。当前内置技能的处理函数均由 MockArm 实现。

> 当前阶段：**技能驱动的 Mock 执行链路**。项目不会连接或控制真实机械臂，请勿将当前实现直接用于生产设备。

## 当前能力

已实现并接入主链路的能力：

- `Command`、`SafetyResult`、`ExecutionResult` 等命令数据模型
- 基于 `SkillDefinition` 和 `ParamSpec` 的技能定义，每个技能可注册独立 `handler`
- 内置技能注册、查询及启用状态检查
- 必需参数、参数类型和额外参数校验
- 基于 `ParamSpec` 的通用数值边界安全检查，支持开区间和闭区间
- Command Gateway 统一编排、技能 `handler` 分发及执行异常转换
- MockArm 模拟执行
- JSON 命令解析和基础结构校验
- 交互式命令行入口
- 核心链路测试

以下目录或文件目前仍是占位骨架，尚未接入运行链路：

- 真实机械臂 Adapter、运动学与 ROS 2
- Web API 与前端页面
- LLM 客户端、Prompt 和自然语言模型调用
- RAG、状态机、结构化日志和自动评估
- YAML 配置加载
- Docker、运行脚本及 Python 包构建配置

## 执行流程

`run_command(command, skills)` 当前按下面的顺序处理命令：

```text
Command
  -> Skill Registry：技能是否存在
  -> Enabled Check：技能是否启用
  -> Params Validator：参数是否符合技能定义
  -> Safety Checker：参数是否满足 ParamSpec 安全边界
  -> Skill Handler：调用技能注册的处理函数
  -> ExecutionResult
```

```mermaid
flowchart TD
    A[Command] --> B[查找 SkillDefinition]
    B -->|不存在| R[返回失败 ExecutionResult]
    B -->|存在| C{技能已启用?}
    C -->|否| R
    C -->|是| D[校验参数 Schema]
    D -->|失败| R
    D -->|通过| E[Safety Checker]
    E -->|不安全| R
    E -->|安全| F[调用 Skill Handler]
    F -->|正常返回| G[返回 ExecutionResult]
    F -->|抛出异常| H[转换为失败 ExecutionResult]
```

## 内置技能

| 技能 | 风险等级 | 默认状态 | 参数 | 当前 Handler | 作用 |
| --- | --- | --- | --- | --- | --- |
| `move_arm` | `medium` | 启用 | `x`、`y`、`z` | `mock_arm.move_arm` | 移动到指定笛卡尔坐标 |
| `open_gripper` | `low` | 启用 | 无 | `mock_arm.open_gripper` | 打开夹爪 |
| `close_gripper` | `low` | 启用 | 无 | `mock_arm.close_gripper` | 关闭夹爪 |
| `stop` | `low` | 启用 | 无 | `mock_arm.stop` | 停止当前动作 |

所有内置技能默认不允许额外参数。`move_arm` 的 `x`、`y`、`z` 必须是 `int` 或 `float`，并满足：

```text
0 < x, y, z <= 1
```

`validate_skill_params()` 负责必需字段、精确类型和额外字段校验；`Safety Checker` 会再次防御性检查必需字段和类型，并通用地执行 `min_value`、`max_value`、`min_inclusive` 和 `max_inclusive` 定义的边界规则。

## 核心数据结构

### Command

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `command_id` | `str` | 命令唯一标识 |
| `skill_name` | `str` | 要执行的技能名称 |
| `params` | `dict` | 技能参数 |
| `source` | `str` | 命令来源，如 `user` |

### SkillDefinition

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `skill_name` | `str` | 技能名称 |
| `description` | `str` | 技能说明 |
| `risk_level` | `str` | 风险等级 |
| `enabled` | `bool` | 是否启用 |
| `params_schema` | `dict[str, ParamSpec]` | 参数定义 |
| `handler` | `Callable[[Command], ExecutionResult]` | 通过所有检查后调用的技能处理函数 |
| `allow_extra_params` | `bool` | 是否允许未声明参数，默认 `False` |

### ParamSpec

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `accepted_types` | `tuple[type, ...]` | 允许的精确参数类型 |
| `required` | `bool` | 是否必需，默认 `True` |
| `min_value` / `max_value` | `float \| None` | 可选数值下界与上界 |
| `min_inclusive` / `max_inclusive` | `bool` | 边界是否包含端点，默认 `True` |

### ExecutionResult

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `command_id` | `str` | 对应的命令 ID |
| `skill_name` | `str` | 技能名称 |
| `success` | `bool` | 是否成功 |
| `message` | `str` | 执行结果或拒绝原因 |

## 环境要求

- Python 3.10 或更高版本
- 当前已接入的主链路只使用 Python 标准库

项目采用 `src/` 布局，但 `pyproject.toml` 和 `requirements.txt` 目前为空，因此尚不能通过常规方式安装。运行命令时需要设置 `PYTHONPATH=src`。

## 快速开始

### 1. 获取项目并进入目录

```bash
cd rosclaw-mini
```

### 2. 创建虚拟环境（可选）

使用 Conda：

```bash
conda create -n rosclaw-mini python=3.10
conda activate rosclaw-mini
```

### 3. 启动交互式入口

```bash
PYTHONPATH=src python3 -m rosclaw_mini.main
```

当前入口接收的是 JSON，不会调用大语言模型。示例输入：

```json
{"skill_name": "move_arm", "params": {"x": 0.5, "y": 0.4, "z": 0.3}}
```

其他可用输入：

```json
{"skill_name": "open_gripper", "params": {}}
{"skill_name": "close_gripper", "params": {}}
{"skill_name": "stop", "params": {}}
```

输入 `exit` 退出程序。每条命令会自动生成 UUID，并以 `ExecutionResult` 输出处理结果。

## Python 调用示例

```python
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.skills.registry import BUILTIN_SKILLS

command = Command(
    command_id="cmd-001",
    skill_name="move_arm",
    params={"x": 0.5, "y": 0.4, "z": 0.3},
    source="user",
)

result = run_command(command, BUILTIN_SKILLS)
print(result)
```

预期结果：

```text
ExecutionResult(command_id='cmd-001', skill_name='move_arm', success=True, message="机械臂已移动到位置: {'x': 0.5, 'y': 0.4, 'z': 0.3}")
```

典型失败情况包括：

| 情况 | 示例结果 |
| --- | --- |
| 技能不存在 | `技能不存在: destroy_arm` |
| 技能未启用 | `技能未启用: move_arm` |
| 缺少参数 | `缺少必需参数: z` |
| 包含额外参数 | `不允许额外参数: speed` |
| 坐标越界 | `UnsafeCommand, Parameter x value 2.0 is greater than maximum allowed value 1` |
| Handler 抛出异常 | `技能执行失败: mock arm disconnected` |

## JSON 命令解析

`parse_json_command()` 负责把 JSON 字符串转换为 `Command`。输入对象必须同时包含：

- 非空字符串 `skill_name`
- 字典类型 `params`

```python
from rosclaw_mini.llm.command_parser import parse_json_command

command = parse_json_command(
    '{"skill_name": "open_gripper", "params": {}}',
    "cmd-002",
)
```

非法 JSON 会抛出 `json.JSONDecodeError`，结构不合法会抛出 `ValueError`。该模块目前只是确定性解析器，并未接入 LLM 客户端。

同一模块中的 `parse_command()` 可将“打开夹爪”、“关闭夹爪”和“停止”映射为固定技能，其他文本映射为 `unknown`；该函数目前未接入交互式主入口。

## 测试

安装 `pytest` 后可运行完整测试集：

```bash
PYTHONPATH=src python3 -m pytest -q
```

当前测试覆盖：

- 命令与执行结果数据模型
- 技能定义和注册表查询
- JSON 命令解析及非法输入
- 参数 schema 校验
- 通用安全边界的开闭区间与越界拦截
- 未知技能、禁用技能和正常执行
- 自定义 Handler 分发与 Handler 异常转换
- MockArm 的四个内置处理函数

## 项目结构

```text
rosclaw-mini/
├── README.md
├── pyproject.toml                 # 当前为空，待补充包配置
├── requirements.txt              # 当前为空，主链路无第三方依赖
├── configs/                       # YAML 配置占位
├── docs/
│   └── arm_actions.md             # 机械臂动作接口设计文档
├── scripts/                       # 运行脚本占位
├── src/rosclaw_mini/
│   ├── main.py                    # JSON 命令交互入口
│   ├── command_schema/
│   │   ├── commands.py            # 命令、安全和执行结果模型
│   │   └── schemas.py             # 早期服务网关模型
│   ├── skills/
│   │   ├── base.py                # SkillDefinition / ParamSpec / SkillHandler
│   │   ├── arm_skills.py          # 内置技能与 Handler 绑定
│   │   ├── registry.py            # 技能注册表
│   │   └── validator.py           # 通用参数校验
│   ├── gateway/command/gateway.py # 命令执行编排
│   ├── safety/checker.py          # 通用 ParamSpec 安全边界检查
│   ├── arm/mock_arm.py            # 四个模拟技能 Handler
│   ├── llm/
│   │   ├── command_parser.py      # 文本/JSON 确定性解析
│   │   └── command_validator.py   # JSON 数据结构校验
│   ├── evaluation/                # 占位
│   ├── logging/                   # 占位
│   ├── rag/                       # 占位
│   ├── ros2/                      # 占位
│   ├── state/                     # 占位
│   └── web/                       # 占位
└── tests/
```

## 当前边界与注意事项

1. **内置技能只支持模拟执行**：Gateway 已支持按技能注册不同 Handler，但内置 Handler 仍全部来自 `MockArm`，只返回预设结果，不提供真实运动、碰撞检测或硬件反馈。
2. **参数校验职责有部分重复**：`validate_skill_params()` 与 `Safety Checker` 都会检查必需参数和类型；数值边界只由 `Safety Checker` 从 `ParamSpec` 通用读取。
3. **配置尚未生效**：`configs/*.yaml` 为空，技能和安全限制都定义在 Python 源码中。
4. **风险等级仅作元数据**：Gateway 目前不会根据 `risk_level` 动态授权或执行额外策略。
5. **LLM 尚未接入**：主入口要求用户提供结构化 JSON；`llm/client.py` 和 `llm/prompts.py` 仍为空。
6. **包配置尚未完成**：当前依赖 `PYTHONPATH=src` 运行，pytest 也未写入项目依赖。
7. **早期服务网关模型未接入主链路**：`command_schema/schemas.py` 和 `notebooks/` 中的内容属于探索性原型。

## 建议的下一步

1. 明确区分参数结构校验与安全校验的职责，减少必需参数和类型检查的重复
2. 补齐 `pyproject.toml`、开发依赖和 pytest 配置
3. 将技能开关与安全边界迁移到 YAML 配置
4. 为真实 Adapter 实现技能 Handler，并增加超时、重试和硬件错误分类
5. 增加结构化日志、命令状态跟踪和更多边界测试
6. 在安全链路稳定后接入 ROS 2、Web API 和 LLM
