---
document_id: safety-rules
title: Validator、Safety Checker 与运动安全边界
category: safety
source: rosclaw-mini-source
version: "1.0"
risk_level: high
tags:
  - validator
  - safety
  - workspace
  - TCP
  - MuJoCo
  - calibration
source_files:
  - src/rosclaw_mini/skills/validator.py
  - src/rosclaw_mini/safety/checker.py
  - src/rosclaw_mini/safety/limits.py
  - src/rosclaw_mini/workspace_scan/irregular_workspace.py
  - src/rosclaw_mini/arm/so100_plus_session.py
  - src/rosclaw_mini/arm/so100_plus_trajectory_validation.py
  - tests/test_safety_checker.py
---

# Validator、Safety Checker 与运动安全边界

## 分层检查

Skill Validator 检查 params 是字典、必填参数、Python 类型及多余参数。
Safety Checker 读取同一 `SkillDefinition.params_schema` 的上下限，拒绝 bool、
NaN 和无穷；`move_relative` 还拒绝三轴全零。Handler/Session 再执行依赖
实际状态的工作空间、IK、关节、碰撞、接触、跟踪和运行期检查。

任何 RAG 文本或 LLM 输出都不能修改这些检查。知识片段写着“安全”不代表
当前命令已安全，最终结论只能来自当次运行的代码。

## 坐标、单位与 TCP

运动坐标统一用米。绝对和相对运动都控制运动学模型的夹爪 TCP，坐标原点
和轴属于底座固定坐标系：`+X` 向前/伸出，`+Z` 向上；Prompt 的操作者
默认观察约定把左映射为 `+Y`、右映射为 `-Y`。模型不支持用户指定任意
末端 roll/pitch/yaw。

相对运动的当前 TCP 必须在 Handler 真正开始执行时读取，不能由 LLM、
知识库或启动缓存提供。

## SO-100 Plus 正式工作空间

真机普通 WORK 运动使用已保存的不规则网格，不是整个 AABB 长方体。
目标需要通过网格有效节点/完整单元与规划通道，完整最终执行 waypoint
需要经过 MuJoCo、关节和接触预检；执行前还会核对真实起点。工作空间与
`/dev/lerobot_right`、follower `right`、校准文件及 SHA-256 认证绑定。

静态工作空间不表示现场没有人员、线缆或障碍物，也不代表机械臂当前就在
WORK。实时 TCP、关节、夹爪和会话状态必须从运行时读取。
