---
document_id: testing-and-operations
title: 测试、运行与故障排查
category: operations
source: rosclaw-mini-source
version: "1.0"
risk_level: high
tags:
  - pytest
  - mock
  - operations
  - troubleshooting
  - error
source_files:
  - pyproject.toml
  - src/rosclaw_mini/main.py
  - src/rosclaw_mini/runtime.py
  - tests/test_main.py
  - tests/test_llm_command_pipeline.py
  - tests/test_runtime.py
  - README.md
---

# 测试、运行与故障排查

## 无硬件验证

默认入口 `PYTHONPATH=src python -m rosclaw_mini.main` 使用 JSON + Mock。
自然语言和 RAG 首次验证使用 `--input-mode llm --backend mock`。自动测试
使用 Fake/Mock，不应访问 `/dev`、连接 Robot、启用电机或发起真实 LLM
HTTP 请求。pytest 配置把 `src` 加入 pythonpath。

## 常见输入与结果

CLI 的 `result` 查询后台最后结果，`exit` 进入 Runtime 安全关闭；它们是
入口控制词，不经 LLM。普通命令忙碌时新普通命令不会提交，明确 stop
仍可走独立通道。LLM 网络错误、非法响应 JSON 和不合法 Command 会显示
明确错误并继续接收输入。

## 失败定位顺序

先看 `ExecutionResult.message`：技能不存在/禁用通常属于 Registry；缺参、
类型或额外参数属于 Validator；非有限值和粗范围属于 Safety Checker；
状态、工作空间、IK、MuJoCo、跟踪、过载和温度属于 Handler/Session/
Adapter。UNVERIFIED 表示实际姿态不能继续信任，不等于进程崩溃。

## 静态知识限制

知识库适合解释协议和项目约束，不提供实时 TCP、关节、夹爪、温度、电流、
障碍物或会话状态。代码、测试和正式文档变化后必须同步维护知识来源；
若来源冲突，以当前可执行代码和测试行为为准。
