---
document_id: arm-backends
title: Mock 与 SO-100 Plus 后端职责
category: hardware
source: rosclaw-mini-source
version: "1.0"
risk_level: high
tags:
  - MockArmAdapter
  - SO100PlusAdapter
  - ArmHandlers
  - backend
  - calibration
source_files:
  - src/rosclaw_mini/arm/base.py
  - src/rosclaw_mini/arm/mock_arm.py
  - src/rosclaw_mini/arm/so100_plus.py
  - src/rosclaw_mini/arm/so100_plus_factory.py
  - src/rosclaw_mini/skills/arm_handler.py
  - src/rosclaw_mini/runtime.py
  - tests/test_mock_arm_adapter.py
  - tests/test_so100_plus_factory.py
---

# Mock 与 SO-100 Plus 后端职责

## Adapter 与 Handler

`ArmAdapter` 暴露连接、TCP 读取、绝对/相对移动、夹爪、stop、卸力和断开
等原子能力。`ArmHandlers` 把 Command 参数映射到 Adapter，并返回统一
`ExecutionResult`；Adapter 不解析 LLM 文本，也不决定 Gateway 协议。

## Mock 后端

`MockArmAdapter` 只维护内存位置和夹爪状态，不访问 `/dev`。它适合先验证
JSON、RAG、LLM、Parser、Validator、Gateway、Controller 和 stop 链路。
Mock 工作空间 `[-1, 1] m` 只是演示范围，不能当成真机数据。

## SO-100 Plus 后端

真机必须显式使用 `--backend so100_plus` 和风险确认参数。Factory/Runtime
只认证 `/dev/lerobot_right`、follower `right` 和指定校准哈希。连接会
启用力矩并恢复已保存运行参数。普通 WORK Adapter 与 REST/WORK 固定转换
Adapter 职责分离，不能用转换外包框绕过正式不规则工作空间。

摄像头是独立可选功能，不是机械臂连接条件；当前 RAG 命令链不会自动
打开摄像头、连接真机或执行维护脚本。
