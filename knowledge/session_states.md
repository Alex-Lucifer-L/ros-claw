---
document_id: session-states
title: 机械臂会话状态
category: state
source: rosclaw-mini-project
version: "1.0"
risk_level: high
tags:
  - state
  - safety
  - workflow
---

# 机械臂会话状态

机械臂会话状态用于描述当前机械臂是否具备执行运动命令的条件。

## REST

REST 表示机械臂处于收纳或休息状态。

在 REST 状态下，不允许直接执行普通的绝对移动或相对移动命令。机械臂需要先通过规定的展开流程进入 WORK 状态。

## TRANSITION

TRANSITION 表示机械臂正在 REST 与 WORK 之间进行状态切换。

在 TRANSITION 状态下，不接受新的普通运动命令，避免多个运动流程发生冲突。

## WORK

WORK 表示机械臂已经进入经过确认的工作状态。

只有在 WORK 状态下，才允许继续检查并执行普通的绝对移动或相对移动命令。具体目标仍然必须通过参数校验、工作空间检查和安全检查。

## UNVERIFIED

UNVERIFIED 表示程序无法确认机械臂当前的真实状态。

发生跟踪误差、运动中断、通信异常或状态同步失败后，机械臂应进入 UNVERIFIED 状态。

在 UNVERIFIED 状态下，不允许继续执行普通运动命令。必须先重新确认机械臂状态，才能恢复执行。

## 安全原则

状态知识用于帮助 LLM 理解机械臂的操作流程，但不能代替代码中的状态门禁、安全检查和运行时保护。