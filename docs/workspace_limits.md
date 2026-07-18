# SO-100 Plus 工作空间限制

## 正式范围

当前 `right_follower` 登记的正式 TCP 可达工作空间是三轴闭区间：

```text
X:  0.3135714232672181 .. 0.4335714232672181 m
Y: -0.041185494280163625 .. 0.018814505719836373 m
Z:  0.17932848288990053 .. 0.29932848288990055 m
```

尺寸约为 `12 × 6 × 12 cm`。代码中的唯一来源是：

```python
from rosclaw_mini.safety.limits import (
    SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
)
```

这个范围来自 `14 × 8 × 14 cm` JoyCon 初始 TCP 姿态仿真候选框，
六个面各内缩一个 `1 cm` 网格。14 个内缩边界代表点都已经进行真机
运动测试：12 个满足 `12 mm` 到位门槛，2 个安全到达但误差约为
`24.800 mm` 和 `14.780 mm`。测试期间没有路径碰撞、过载或过温。

因此“正式”表示项目允许把范围内坐标作为当前机械臂的可达目标，
不表示范围内每一点都保证 `12 mm` 定位精度。

## 接入 Skill

使用专用构造函数时，`move_arm` 会自动启用，并把正式边界写入
Validator 和 Safety Checker：

```python
from rosclaw_mini.skills.arm_skills import (
    build_so100_plus_right_follower_arm_skills,
)

skills = build_so100_plus_right_follower_arm_skills(adapter)
```

低于或高于任一闭区间边界的命令会在到达 Adapter 前被拒绝。通用
`build_arm_skills(adapter)` 仍然失败关闭；没有明确选择这台机械臂时，
不会自动套用真机范围。

## 接入 Adapter

Adapter 的运动学检查应使用同一范围：

```python
from rosclaw_mini.safety.limits import (
    build_so100_plus_right_follower_motion_limits,
)

motion_limits = build_so100_plus_right_follower_motion_limits(
    current_joint_radians,
)
```

该函数还会组合：

- 当前 `right_follower` 实测底座旋转范围；
- 第三方模型的其余关节范围；
- 默认 `2°` 规划内部关节步长。

## 适用条件

这组范围只适用于：

- 当前 `right_follower` 和 `right_follower.json` 校准；
- 当前安装底座；
- 桌面与底座底部齐平，TCP 不低于该平面；
- 工作空间内没有新增障碍物、人体或线缆干涉；
- JoyCon 初始 TCP 姿态附近，`move_to()` 保持当前姿态。

更换机械臂、校准、底座、TCP 工具偏移或工位后，必须重新验证。它也
不是任意末端姿态下的全局工作空间，不能替代每次 IK、关节、MuJoCo
路径、负载、温度和跟踪误差检查。
