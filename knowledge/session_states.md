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
  - unfold_arm
  - fold_arm
  - revalidate_state
source_files:
  - src/rosclaw_mini/arm/so100_plus_session.py (ArmSessionState, SO100PlusArmSession)
  - src/rosclaw_mini/skills/arm_skills.py (bind_so100_plus_arm_session)
  - src/rosclaw_mini/runtime.py (build_so100_plus_runtime)
  - tests/test_so100_plus_session.py
---

# 机械臂会话状态

机械臂会话状态用于描述当前机械臂是否具备执行运动命令的条件。

## REST

REST 表示真实关节反馈已经通过 `follower_rest` 收纳姿态及其容差检查，
不是“程序空闲”的同义词。

在 REST 状态下允许 `unfold_arm` 和 `stop`。普通绝对移动
`move_arm`、相对移动 `move_relative`、夹爪动作和 `fold_arm` 均由
`SO100PlusArmSession` 拒绝。进入 WORK 必须执行无用户路径参数的
`unfold_arm` 认证过渡。

## TRANSITION

TRANSITION 表示机械臂正在 REST 与 WORK 之间进行状态切换。

在 TRANSITION 状态下只允许 `stop`。展开或收纳已经开始后发生异常或
中断，会话进入 UNVERIFIED，不能按原计划目标虚假推断状态。

## WORK

WORK 表示机械臂已到达 `middle_internal`，或当前真实姿态通过正式不规则
工作空间、关节、底座、夹爪映射和 MuJoCo 静态姿态门禁。

只有在 WORK 状态下，才允许继续检查 `move_arm`、`move_relative`、
`open_gripper`、`close_gripper` 和 `fold_arm`。WORK 不是一次通用安全
批准；每个新目标仍必须通过 Validator、Safety Checker、不规则工作空间、
IK、关节限制、MuJoCo 轨迹和运行期保护。

## UNVERIFIED

UNVERIFIED 表示程序无法确认机械臂当前的真实状态。

发生已写入运动后的跟踪失败、转换中断、通信异常或姿态无法认证时，
机械臂进入 UNVERIFIED。

在 UNVERIFIED 状态下只允许 `stop`、安全退出和只读
`revalidate_state`。`revalidate_state` 不发送运动：它重新读取实际反馈，
只有姿态确实符合 REST 或 WORK 的全部门禁时才恢复相应状态。

## 认证工作流

正常真机会话是 `REST → unfold_arm → WORK → 工作动作 → fold_arm → REST`。
`unfold_arm` 与 `fold_arm` 使用生产模块中的固定、预检轨迹，用户不能通过
Command 参数覆盖中间点、速度或边界。程序启动时会读取真实反馈分类为
REST、WORK 或 UNVERIFIED，不会默认机械臂处于 follower_rest。

## 安全原则

状态知识用于帮助 LLM 理解机械臂的操作流程，但不能代替代码中的状态门禁、安全检查和运行时保护。
