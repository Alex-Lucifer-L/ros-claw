---
document_id: gateway-and-permissions
title: Gateway 顺序与权限脚手架边界
category: gateway
source: rosclaw-mini-source
version: "1.0"
risk_level: high
tags:
  - Gateway
  - registry
  - permissions
  - routing
  - failure
source_files:
  - src/rosclaw_mini/gateway/command/gateway.py
  - src/rosclaw_mini/skills/registry.py
  - src/rosclaw_mini/skills/validator.py
  - src/rosclaw_mini/safety/checker.py
  - src/rosclaw_mini/command_schema/schemas.py
  - tests/test_gateway.py
---

# Gateway 顺序与权限脚手架边界

## 正式命令 Gateway

`run_command(command, skills)` 的实际顺序是：按名称查找 Skill；检查
`enabled`；验证 params；运行通用 Safety Checker；调用 Skill Handler；
把 Handler 异常转成失败 `ExecutionResult`。不存在的 Skill 返回
“技能不存在”，禁用 Skill 返回“技能未启用”。RAG 不在此顺序中增加
放行分支。

## Registry 与风险等级

当前 Registry 是一次 Runtime 持有的 `dict[str, SkillDefinition]`。
`risk_level` 会进入安全结果和知识说明，但当前代码没有完整的按用户身份、
风险等级动态审批系统。真机风险确认、会话状态和硬件认证由 CLI/Runtime/
Session 的现有门禁实现。

## 尚未接入的路由与权限结构

`command_schema/schemas.py` 定义了 `GatewayRequest`、`Route`、`Service`、
`Permission` 和 `GatewayResponse` 数据类，但它们目前没有接入上述正式
Command 执行函数，也没有 HTTP 服务路由或权限判定实现。LLM 不得根据
这些脚手架宣称用户已经认证或某路由可以使用。
