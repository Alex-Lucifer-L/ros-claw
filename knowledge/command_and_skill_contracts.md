---
document_id: command-and-skill-contracts
title: Command 数据结构与正式 Skill 契约
category: protocol
source: rosclaw-mini-source
version: "1.0"
risk_level: high
tags:
  - Command
  - SkillDefinition
  - move_arm
  - move_relative
  - gripper
  - stop
source_files:
  - src/rosclaw_mini/command_schema/commands.py
  - src/rosclaw_mini/skills/base.py
  - src/rosclaw_mini/skills/arm_skills.py
  - src/rosclaw_mini/llm/command_parser.py
  - tests/test_skill_registry.py
  - tests/test_skill_validator.py
---

# Command 数据结构与正式 Skill 契约

## Command 与结果

`Command` 包含 `command_id`、`skill_name`、`params` 和 `source`。
LLM 只输出 `{"skill_name":"...","params":{...}}`；Parser 在程序内补充
唯一 ID 和 `source="user"`。`SafetyResult` 记录安全结论、风险和原因；
`ExecutionResult` 记录命令 ID、Skill、成功标志和消息。

## 普通 Skill

- `move_arm`：medium，必填 `x/y/z`，单位米，表示基座坐标系 TCP 绝对目标。
- `move_relative`：medium，必填 `dx/dy/dz`，单位米；执行时读取当前 TCP，
  合成绝对目标并复用普通绝对运动安全链。
- `open_gripper`：low，无参数，打开夹爪。
- `close_gripper`：low，无参数，关闭夹爪。
- `stop`：low，无参数，通过 Controller 的独立 `request_stop()` 路径处理。
- `disable_torque`：high，无参数，但默认 `enabled=False`，不会提供给普通
  Gateway 或 LLM 命令。

没有显式工作空间时，`move_arm` 和 `move_relative` 默认不启用。

## SO-100 Plus 会话 Skill

绑定真机会话后额外提供：`unfold_arm`（high，无参数）、`fold_arm`
（high，无参数）和 `revalidate_state`（low，无参数、只读）。这些 Skill
不接受用户提供的路径、关节角、中间点、速度或安全阈值。

## 参数协议

`ParamSpec` 描述类型、必填性、上下限和区间开闭。默认不允许额外参数。
数值参数不接受 bool、NaN 或无穷。LLM 不得猜缺失距离、TCP、关节反馈
或传感器值；无法准确表达的请求应输出保留的 `unsupported_action`，由
Gateway 按“技能不存在”失败关闭，而不是发明新协议。
