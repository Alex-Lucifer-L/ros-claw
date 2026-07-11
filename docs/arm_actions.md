# Arm Actions Protocol V1

本文档规定 RosClaw Mini 在 MockArm 模拟阶段支持的机械臂技能、LLM 输出格式、参数要求和安全限制。

> 当前协议只描述已经接入 Parser、Validator、Safety Checker、Skill Registry、Gateway 和 MockArm 的技能。接入真实机械臂后，坐标系、单位和安全范围必须根据真实驱动重新确定。

## 通用命令格式

LLM 只负责生成 `skill_name` 和 `params`，并且必须输出纯 JSON，不能附加解释文字、Markdown 代码围栏或 JSON 注释。

```json
{
  "skill_name": "open_gripper",
  "params": {}
}
```

### LLM 输出字段

| 字段 | JSON 类型 | 是否必需 | 含义 |
| --- | --- | --- | --- |
| `skill_name` | string | 是 | 要调用的技能名称，必须与当前支持的技能名完全一致 |
| `params` | object | 是 | 技能所需参数；无参数技能也必须提供空对象 `{}` |

LLM 不得生成 `command_id` 和 `source`。

### 系统生成字段

| 字段 | 生成者 | 含义 |
| --- | --- | --- |
| `command_id` | 系统入口 | 使用 UUID 唯一标识每一条命令 |
| `source` | 系统 | 记录命令的原始来源，当前固定为 `user` |

系统完成 JSON 解析和结构验证后，使用 LLM 输出字段与系统生成字段构造完整的 `Command` 对象。

## 当前支持的技能

| `skill_name` | 功能 | 风险等级 |
| --- | --- | --- |
| `move_arm` | 移动机械臂到指定目标位置 | medium |
| `open_gripper` | 打开机械臂夹爪 | low |
| `close_gripper` | 关闭机械臂夹爪 | low |
| `stop` | 停止当前机械臂动作 | low |

不在此列表中的技能当前必须被系统拒绝。

## `move_arm`

### 功能

将机械臂移动到指定目标位置。当前 `move_arm` 表示绝对目标位置，不表示相对移动距离。

### 参数要求

| 参数 | 类型 | 是否必需 | 含义 | 当前安全范围 |
| --- | --- | --- | --- | --- |
| `x` | int 或 float | 是 | 目标位置的 x 坐标 | `0 < x <= 1` |
| `y` | int 或 float | 是 | 目标位置的 y 坐标 | `0 < y <= 1` |
| `z` | int 或 float | 是 | 目标位置的 z 坐标 | `0 < z <= 1` |

### 单位与坐标系

当前 MockArm 使用临时模拟坐标，尚未绑定真实物理单位。

接入真实机械臂后，必须根据驱动文档重新确定：

- 坐标系原点
- 坐标轴方向
- 长度单位
- 实际工作空间
- 机械臂自身限制和环境安全边界

### 正确示例

```json
{
  "skill_name": "move_arm",
  "params": {
    "x": 0.5,
    "y": 0.4,
    "z": 0.3
  }
}
```

### 错误示例：参数越界

```json
{
  "skill_name": "move_arm",
  "params": {
    "x": 1.5,
    "y": 0.4,
    "z": 0.3
  }
}
```

`x=1.5` 超出当前安全范围，命令应由 Safety Checker 拒绝。

### 错误示例：缺少参数

```json
{
  "skill_name": "move_arm",
  "params": {
    "x": 0.5,
    "y": 0.4
  }
}
```

`params` 缺少必需的 `z` 参数，命令应被拒绝。

### 风险等级

`medium`

## `open_gripper`

### 功能

打开机械臂夹爪。

### 参数要求

该技能不需要内部参数，但 `params` 字段必须存在，并使用空对象 `{}`。

### 正确示例

```json
{
  "skill_name": "open_gripper",
  "params": {}
}
```

### 风险等级

`low`

## `close_gripper`

### 功能

关闭机械臂夹爪。

### 参数要求

该技能不需要内部参数，但 `params` 字段必须存在，并使用空对象 `{}`。

### 正确示例

```json
{
  "skill_name": "close_gripper",
  "params": {}
}
```

### 风险等级

`low`

## `stop`

### 功能

停止机械臂当前正在执行的动作。

### 参数要求

该技能不需要内部参数，但 `params` 字段必须存在，并使用空对象 `{}`。

### 正确示例

```json
{
  "skill_name": "stop",
  "params": {}
}
```

### 特殊说明

`stop` 表示停止机械臂动作，不表示退出 RosClaw Mini 命令行程序。当前命令行程序使用 `exit` 退出。

### 风险等级

`low`

## 后续计划

以下技能尚未实现，当前不允许 LLM 输出：

- `pick`
- `home`
- `get_status`
- `reject`
- `move_relative`
- `move_joint`

只有在 Command Schema、Validator、Safety Checker、Skill Registry、执行器和测试均支持后，技能才能加入“当前支持的技能”列表。
