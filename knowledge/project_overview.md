---
document_id: project-overview
title: 项目目标与命令执行链
category: architecture
source: rosclaw-mini-source
version: "1.0"
risk_level: medium
tags:
  - architecture
  - command
  - gateway
  - llm
  - rag
source_files:
  - src/rosclaw_mini/main.py
  - src/rosclaw_mini/runtime.py
  - src/rosclaw_mini/llm/command_generator.py
  - src/rosclaw_mini/gateway/command/gateway.py
  - src/rosclaw_mini/execution/controller.py
  - tests/test_main.py
---

# 项目目标与命令执行链

rosclaw-mini 把结构化 Command 通过同一条受检链路发送给 Mock 或
SO-100 Plus 后端。默认入口是 `json + mock`；`llm` 只改变 Command 的
产生方式，`so100_plus` 必须显式选择并确认风险。

## JSON 链路

`main.run_json_command_loop()` 解析单个 JSON 对象，构造 `Command`，再交给
`ExecutionController`。后台线程调用 `gateway.command.gateway.run_command()`，
按 Skill 查找、启用状态、参数 Validator、Safety Checker、Handler 的顺序
执行，最终返回 `ExecutionResult`。

## LLM 与 RAG 链路

LLM 模式先用用户原始文本检索静态项目知识，把少量带来源片段加入现有
Command Prompt，然后通过 OpenAI-compatible 客户端取得一个 JSON 对象。
该 JSON 仍由原有 Parser 转为 `Command`，并经过与 JSON 模式完全相同的
Controller、Gateway、Validator、Safety 和后端链路。

RAG 和 LLM 不执行 Skill、不做安全批准，也不能创建 Registry 中不存在的
Skill。检索失败可以退回基础 Prompt；机械臂安全检查不能因此降级。

## 装配边界

`runtime.py` 负责创建 Adapter、Skills 和 Controller。Mock 仅操作内存；
SO-100 Plus Runtime 会验证端口、follower、校准哈希、模型和工作空间，
然后创建普通 WORK Adapter、Transition Adapter 和唯一会话状态持有者。
程序关闭由 Runtime 负责 stop、等待后台线程及断开，不由 LLM 决定。
