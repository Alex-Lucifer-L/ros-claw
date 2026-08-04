---
document_id: execution-and-stop
title: 后台执行、stop 与关闭语义
category: execution
source: rosclaw-mini-source
version: "1.0"
risk_level: high
tags:
  - ExecutionController
  - thread
  - Lock
  - Event
  - stop
  - shutdown
source_files:
  - src/rosclaw_mini/execution/controller.py
  - src/rosclaw_mini/runtime.py
  - src/rosclaw_mini/arm/so100_plus.py
  - src/rosclaw_mini/arm/so100_plus_session.py
  - src/rosclaw_mini/main.py
  - tests/test_execution_controller.py
  - tests/test_stop_integration.py
---

# 后台执行、stop 与关闭语义

## 单后台命令

`ExecutionController.submit()` 在 Lock 下拒绝并发普通命令、初始化新动作
stop 世代并注册 worker，然后在线程中运行 Gateway。Controller 同时只
运行一个普通命令，结束后保存最后一个 `ExecutionResult`；CLI 的
`result` 读取该结果。忙碌时第二条普通命令直接拒绝，不排队、不覆盖
worker/Command/结果；CLI 会显示当前 `command_id`，只有 `result` 和
`stop` 继续放行。

## stop 中断

`stop` 不能走普通 `submit()`，必须调用 `request_stop()`，这样主输入循环
无需等待后台运动返回。真机底层用 Event 通知执行计划，收到后不再发送
后续 waypoint；若已经写入目标则读取当前位置并保持。动作提交后到达的
stop 不会被底层 clear 掉，首条写入前到达时应零运动中断。

转换运动被 stop 后进入 UNVERIFIED，不能根据原目标推断 REST 或 WORK。
软件 stop 不是独立硬件急停，不能替代操作者断电能力。

## 退出与断开

Runtime shutdown 先 stop，再有限等待后台线程。线程仍使用硬件时不能
提前 disconnect；超时后保留延迟清理机会，线程最终结束后再断开。
退出不会擅自展开或收纳。SO-100 Plus 普通 `exit` 只在认证 REST 状态
放行；WORK、TRANSITION、UNVERIFIED 会拒绝普通退出并留在输入循环。
显式 `emergency_exit`/“紧急退出”或 Ctrl+C 会请求 stop，等待后台动作
结束，再紧急关闭力矩并断开，同时警告姿态可能没有回到 REST。RAG 和
LLM 不参与关闭顺序。
