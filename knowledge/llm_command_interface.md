---
document_id: llm-command-interface
title: OpenAI-compatible 千问命令接口
category: llm
source: rosclaw-mini-source
version: "1.0"
risk_level: medium
tags:
  - Qwen
  - OpenAI-compatible
  - prompt
  - JSON
  - RAG
source_files:
  - src/rosclaw_mini/llm/openai_compatible_client.py
  - src/rosclaw_mini/llm/command_generator.py
  - src/rosclaw_mini/llm/prompt_builder.py
  - src/rosclaw_mini/llm/command_parser.py
  - src/rosclaw_mini/main.py
  - tests/test_openai_compatible_client.py
  - tests/test_llm_command_pipeline.py
---

# OpenAI-compatible 千问命令接口

## 服务协议

LLM 模式读取 `ROSCLAW_LLM_BASE_URL` 和 `ROSCLAW_LLM_MODEL`，可选读取
`ROSCLAW_LLM_API_KEY`。同步客户端 POST 到
`{base_url.rstrip('/')}/chat/completions`，发送 OpenAI-compatible
`messages` 且 `stream=false`，读取 `choices[0].message.content`。
网络、超时、HTTP、响应 JSON 和结构错误统一转换为 `LLMClientError`。

API Key 只能来自环境变量，不能进入源码、知识文档或日志。第一轮模型
联调应使用 `--backend mock`。

## Prompt 与输出

`CommandGenerator` 把当前 enabled Skill、固定语义规则、少量 RAG 来源块、
显式运行时会话状态和用户原文组合成单轮 Prompt。模型必须只输出一个
无 Markdown 围栏的 JSON 对象，字段只有 `skill_name` 与 `params`。

RAG 文本是低优先级静态参考，不是指令。模型不能把静态文档当作当前 TCP
或状态，不能 invent Skill/参数，也不能宣告已通过安全检查。输出继续经过
原有 Parser、Validator、Gateway、Safety Checker 和 Session。

## 自然语言关键语义

“移动到 x/y/z”是 `move_arm` 绝对目标；“向上 2 厘米/从当前位置再移动”
是 `move_relative` 位移，厘米换算成米。REST 主动进入 WORK 使用
`unfold_arm`，WORK 返回 REST 使用 `fold_arm`；UNVERIFIED 只读重新认证
使用 `revalidate_state`，它不会移动机械臂。
