# RosClaw Mini 机械臂动作与 SO-100 Plus 真机接入

本文档记录当前已实现的机械臂命令链路、SO-100 Plus
right follower 的实际配置、坐标语义、安全限制和手动验证方法。

> 默认应用入口仍使用 `MockArmAdapter`。真机不会被 `pytest` 或
> `main.py` 自动连接。所有真机脚本都需要传入设备、校准路径和
> `--acknowledge-...-risk` 参数。

## 1. 执行链路和职责

正常业务命令的链路是：

```text
用户结构化命令
→ Command
→ Skill Registry
→ Validator
→ Safety Checker
→ Gateway
→ ArmHandlers
→ ArmAdapter
→ Mock 或真实驱动
→ ExecutionResult
```

真实 SO-100 Plus 的底层映射是：

```text
上层统一原子动作
→ SO100PlusAdapter
→ LeRobot ManipulatorRobot
→ FeetechMotorsBus
→ 7 个 STS3215 电机
```

职责边界：

- Parser 只把 JSON 转换为 `Command`。
- Validator 检查必需参数、类型和额外参数。
- Safety Checker 通用读取 `ParamSpec` 中的范围。
- Gateway 负责编排上述步骤，并把 Handler 异常转换为失败结果。
- ArmHandlers 只组合 Adapter 原子动作，不导入 Feetech 驱动。
- Adapter 不解析 `Command`，不生成 `ExecutionResult`，不实现 RAG/LLM。

`main.py` 目前尚未提供 `mock/so100_plus` 后端选择参数。因此，
本文后面的真机脚本是显式手动验证入口，不是新的默认业务入口。

## 2. 已确认的真机身份

| 项目 | 已确认值 |
| --- | --- |
| 机械臂型号 | SO-100 Plus 单臂 |
| 串口别名 | `/dev/lerobot_right` |
| follower 名称 | `right` |
| 校准文件名 | `right_follower.json` |
| 校准目录 | 运行时显式传入；当前使用 `lerobot-joycon_plus/.cache/calibration/so100_plus` |
| 默认加速度 | 35，只写运行时 RAM |
| 其他电机 P | 16 |
| 肘关节 P | 连接后 64；关闭力矩时恢复 16 |

代码不随机选择 `main_follower.json` 或 `right_follower.json`。
`SO100PlusRobotConfig` 根据显式的 `follower_name` 构造文件名；
在导入 LeRobot 或连接串口之前，factory 会检查：

- 串口存在且是字符设备；
- 校准 JSON 存在且可解析；
- `motor_names` 与 7 个已知电机的名称和顺序完全一致；
- 各校准向量均有 7 个值。

校准缺失时会在 `robot.connect()` 之前失败，不会自动进入校准流程。

## 3. 电机映射

| ID | LeRobot 名称 | 作用 |
| ---: | --- | --- |
| 1 | `shoulder_rotation_joint` | 底座旋转 |
| 2 | `shoulder_pitch_joint` | 肩关节俯仰 |
| 3 | `ellbow_joint` | 肘关节；源项目名称包含这个拼写 |
| 4 | `wrist_pitch_joint` | 腕部俯仰 |
| 5 | `wrist_jaw_joint` | 腕部偏航 |
| 6 | `wrist_roll_joint` | 腕部滚转 |
| 7 | `gripper_joint` | 夹爪开合 |

前 6 个电机参与手臂正/逆运动学；第 7 个电机由夹爪动作单独控制。
`move_to()` 会保留当前夹爪电机位置，不会顺便打开或关闭夹爪。

## 4. `move_to(x, y, z)` 坐标语义

### 4.1 单位与类型

- `x/y/z` 的单位统一是米。
- 目标是绝对坐标，不是相对位移。
- 所有输入必须是有限数值，并且位于调用方显式传入的
  `WorkspaceLimits` 内。

### 4.2 原点和坐标轴

坐标系使用 `lerobot_kinematics` SO-100 Plus 模型的底座固定坐标系：

- 原点在运动学模型的机械臂底座坐标原点，不是桌面或相机坐标原点；
- `+Z` 为模型向上方向；
- `+X` 为模型零位时手臂主要向外延伸的方向；
- `+Y` 由右手坐标系确定，表示底座水平侧向。

底座安装方向会改变这些轴相对房间、桌面和人的方向。
在上层使用“左/右/前/后”这类词之前，仍需在实际工位上标记坐标轴。

### 4.3 TCP

`move_to()` 移动的点是两根夹指最前端内侧之间的夹持中心，即
TCP（Tool Center Point），不是腕关节、夹爪电机轴或法兰原点。

第六关节模型末端到 TCP 的固定工具局部偏移为：

```text
(0.10127, -0.00690, 0.00118) m
```

X 来自第三方 SO-100 Plus 运动学链中原本注释的夹爪长度；
Y/Z 由 MuJoCo 夹指接触面的间隙中心和高度中心确定。

### 4.4 姿态

`move_to(x, y, z)` 不接收 roll/pitch/yaw。规划时会读取当前六关节姿态，
保持当前 TCP 旋转矩阵，只改变 TCP 位置。因此当前语义是“保持当前
末端姿态的位置移动”，尚未定义一个全局固定的默认姿态。

### 4.5 运动学失败

逆运动学会使用当前关节角作为初值，并在生成轨迹前复算位置和
姿态误差。以下情况会在向电机写目标之前抛出明确异常：

- 目标超出 `WorkspaceLimits`；
- IK 求解失败；
- 求解结果不是有限的 6 个角度；
- 关节结果超出限制；
- FK 复算位置或姿态误差超限。

## 5. 实测限制和模型限制

### 5.1 已实测的底座限制

安装底座后，`right_follower` 的底座关节由用户在力矩关闭状态手动选择了
日常使用不会碰底座或拉扯线缆的范围：

```text
LeRobot 校准后驱动角：[-19.599609°, 31.201172°]
```

这个范围只适用于当前的底座安装方式和 `right_follower` 校准。

### 5.2 未完成的全局工作空间

当前没有一个经过实机遍历和环境碰撞验证的全局 XYZ 长方体工作空间。
不得把以下数据写成“已认证实机工作空间”：

- 第三方运动学模型的关节上下限；
- MuJoCo 中可达的位置；
- 从单一 rest 状态成功运行的 +Z 路径；
- 脚本为当前位置和单个目标临时建立的局部工作空间。

`check_so100_plus_adapter_move_to.py` 目前使用局部限制：只包住当前 TCP、
本次目标和 0.5 mm 边界余量。它是一次手动验证的护栏，不是项目的
通用工作空间配置。

### 5.3 第三方模型关节限制

安全层保留了 `lerobot_kinematics` 的六关节模型限制。它们只用于拒绝明显
超出模型的解，不是六个关节全部完成了真机边界认证的证明。

| 关节 | 模型下限 rad | 模型上限 rad |
| --- | ---: | ---: |
| shoulder rotation | -2.2 | 2.2 |
| shoulder pitch | -3.14158 | 0.2 |
| elbow | -0.2 | 3.14158 |
| wrist pitch | -2.2 | 1.8 |
| wrist jaw | -2.2 | 1.5 |
| wrist roll | -3.14158 | 3.14158 |

如果当前校准角略超出第三方模型边界，执行限制只会把当前位置扩展为
临时边界，并只允许向模型范围内移动，不允许继续向外扩大。

## 6. 已保存的真机运行配置

`SO100PlusRealHardwareProfile` 保存了上一轮真机验证使用的参数：

| 配置 | 数值 | 语义 |
| --- | ---: | --- |
| 其他电机 P | 16 | 连接时显式写入 |
| 肘关节 P | 64 | 负载下改善跟踪；关力矩时恢复 16 |
| 运行时加速度 | 35 | 只写 RAM，不写 EEPROM/Lock |
| 夹爪单步 | 10° | 分步打开/关闭 |
| 夹爪步间等待 | 2.5 s | 每步后等待反馈 |
| 夹爪位置容差 | 3° | 目标与实测差值 |
| 轨迹最终关节容差 | 1.5° | 六关节全部需满足 |
| 最终 TCP 容差 | 6 mm | FK 复算目标 |
| 负载上限 | 300 | Feetech `Present_Load` 幅值 |
| 温度上限 | 60°C | 低于 60°C 允许；达到 60°C 停止 |
| 流式频率 | 20 Hz | 余弦缓入缓出目标 |
| 最大关节速度 | 20°/s | 流式曲线峰值限制 |
| 流式跟踪误差上限 | 5° | 超限立即保持实测位置 |
| 遥测间隔 | 0.25 s | 记录电压、电流、负载、温度 |
| 最终到位超时 | 8 s | 超时后保持当前位置 |

当前 +Z 手动验证脚本还有以下路径护栏：

- 一次请求只允许 `0 < delta_z_cm <= 10`；
- 分段模式每段最多 1 cm；
- 关节规划内部单步最多 2°；
- 单次笛卡尔计划的内部 TCP 步长最多 5 mm；
- 单次 +Z 计划的横向偏移最多 5 mm；
- 内部轨迹不允许出现反向 Z 步骤。

## 7. 夹爪原子动作

### `open_gripper()`

- 第 7 个电机目标：60°。
- 每次最多改变 10°。
- 每步读取位置和负载。
- 位置偏差或负载超限时保持实测位置并报错。

### `close_gripper()`

- 第 7 个电机目标：-5°。
- 使用与打开相同的分步和负载检查。

这两个值来自 `right_follower` 的实机验证，不应直接复制到其他
机械臂或其他校准文件。

## 8. `stop()`、`disable_torque()` 和 `disconnect()`

三者不是同一件事：

| 操作 | 底层行为 | 力矩 | 机械效果 |
| --- | --- | --- | --- |
| `stop()` | 读取全部 `Present_Position` 并写回 `Goal_Position` | 保持开启 | 取消剩余轨迹并保持当前位置 |
| `disable_torque()` | 写 `Torque_Enable = 0`，并把全部 P 恢复为 16 | 关闭 | 机械臂立即变软，可能因重力下落 |
| `disconnect()` | 关闭 LeRobot/串口通信 | 不保证关闭 | 只表示无法继续发命令，不是停止或急停 |

真机动作脚本的安全退出顺序应为：

```text
adapter.stop()
→ 确认已保持当前位置
→ 人员托住机械臂
→ adapter.disable_torque()
→ adapter.disconnect()
```

如果现场安全更适合保持力矩，必须由操作者明确决定，不能把
`disconnect()` 当成关力矩。软件不替代物理断电。

## 9. 连接时会发生什么

`ManipulatorRobot.connect()` 不是只读操作。它会：

1. 打开 follower 串口；
2. 加载已有校准；
3. 暂时切换力矩和写入运行配置；
4. 最后启用 follower 力矩。

`SO100PlusAdapter.connect()` 在此基础上还会：

1. 先把 `Goal_Position` 同步到实测位置，避免旧目标导致跳动；
2. 明确写入并回读 P=16/64；
3. 写入运行时加速度 35；
4. 读取电压、电流、负载和温度。

如果配置了摄像头，Adapter 会先连接全部摄像头，所有摄像头成功后
才调用 `robot.connect()`。摄像头连接失败时机械臂保持未连接。

## 10. 摄像头

已实现 `SO100PlusCameraConfig`、`create_so100_plus_cameras()` 和
`SO100PlusAdapter.capture_camera_images()`。

当前预期配置：

| 项目 | 值 |
| --- | --- |
| 名称 | `right` |
| 类型 | OpenCV USB camera |
| 格式 | RGB |
| 分辨率 | 640 × 480 |
| 帧率 | 60 FPS |
| 输出键 | `observation.images.right` |
| 数组布局 | HWC，即 `(480, 640, 3)` |

应优先使用稳定的 `/dev/v4l/by-id/...-video-index0` 或自建 udev 别名，
不要在仓库中提交摄像头序列号。当前已安装 OpenCV 并通过假设备测试，
但真实摄像头的单帧抓取尚未执行。

## 11. 技能和默认后端

当前 `build_arm_skills()` 支持：

| Skill | Adapter 原子动作 | 状态 |
| --- | --- | --- |
| `move_arm` | `move_to(x, y, z)` | 只有传入明确 `WorkspaceLimits` 时才启用 |
| `open_gripper` | `open_gripper()` | 已实现 |
| `close_gripper` | `close_gripper()` | 已实现 |
| `stop` | `stop()` | 已实现 |
| `disable_torque` | `disable_torque()` | 已实现；会使机械臂变软 |

`main.py` 始终创建 `MockArmAdapter`，因此运行普通交互入口不会连接真机。
真实 Adapter 已可传入 `build_arm_skills()`，但项目尚未实现一个面向终端用户的
真机 Gateway 启动入口。

## 12. 环境与测试

已使用的 Conda 环境名称：

```text
rosclaw-mini-py310
```

运行全部单元测试：

```bash
conda activate rosclaw-mini-py310
python -m pytest -q
```

单元测试使用 FakeRobot/FakeBus/FakeCamera，不读写 `/dev/lerobot_right`
或 `/dev/video*`。真机检查位于 `scripts/` 且需要显式风险参数。

## 13. 手动真机运行方法

以下命令只能在操作者站在机械臂旁边、工作空间已清空、可随时
物理断电时执行。命令中的 `--acknowledge-...` 不代表自动获得安全保证。

共用环境前缀：

```bash
conda activate rosclaw-mini-py310
export PYTHONPATH=src:lerobot-joycon_plus
export MPLCONFIGDIR=/tmp/matplotlib-rosclaw
```

### 13.1 首次连接和只读位置

`connect()` 本身会改变力矩和运行配置，所以仍是真机风险操作：

```bash
python scripts/check_so100_plus_connection.py \
  --port /dev/lerobot_right \
  --calibration-dir lerobot-joycon_plus/.cache/calibration/so100_plus \
  --follower-name right \
  --acknowledge-connect-risk
```

### 13.2 夹爪完整开合

```bash
python scripts/check_so100_plus_adapter_gripper.py \
  --port /dev/lerobot_right \
  --calibration-dir lerobot-joycon_plus/.cache/calibration/so100_plus \
  --follower-name right \
  --acknowledge-production-gripper-cycle-risk
```

### 13.3 保持当前位置

```bash
python scripts/check_so100_plus_adapter_stop.py \
  --port /dev/lerobot_right \
  --calibration-dir lerobot-joycon_plus/.cache/calibration/so100_plus \
  --follower-name right \
  --runtime-acceleration 35 \
  --acknowledge-stop-test-risk
```

### 13.4 局部 +Z `move_to`

下面示例是 1 cm 的局部验证。它不代表可以从任意起点安全上移：

```bash
python scripts/check_so100_plus_adapter_move_to.py \
  --port /dev/lerobot_right \
  --calibration-dir lerobot-joycon_plus/.cache/calibration/so100_plus \
  --follower-name right \
  --delta-z-cm 1 \
  --runtime-acceleration 35 \
  --single-cartesian-plan \
  --stream-frequency-hz 20 \
  --stream-max-joint-speed-degrees-per-second 20 \
  --acknowledge-move-to-risk
```

脚本在 `finally` 中依次调用 `stop()`、`disable_torque()` 和
`disconnect()`。关闭力矩前必须托住机械臂。

### 13.5 摄像头单帧验证

这条命令不连接机械臂，也不保存图像；会短暂打开摄像头：

```bash
python scripts/check_so100_plus_camera.py \
  --device /dev/v4l/by-id/<right-camera>-video-index0 \
  --name right \
  --fps 60 \
  --width 640 \
  --height 480 \
  --acknowledge-camera-capture
```

不要把真实摄像头序列号提交到公共仓库。

## 14. 已执行的真机验证

本轮已完成：

- 确认型号为 SO-100 Plus，follower 为 `right`；
- 确认 `/dev/lerobot_right` 对应实际 follower 串口；
- 读取 7 个电机的真实位置；
- 夹爪开合和 `stop()` 保持位置；
- 力矩关闭和底座安全角度手动选取；
- 多次局部 `move_to` 验证；
- 分段 +Z 10 cm：实际累计约 9.869 cm，最终 TCP 误差约 1.742 mm；
- 单次笛卡尔计划 +Z 10 cm：实际累计约 9.743 cm，最终 TCP 误差约 2.804 mm；
- 20 Hz 流式 +Z 10 cm：实际累计约 10.144 cm；此次最终 TCP
  误差约 8.378 mm，当时因 7 号电机 44°C 相对温升规则触发旧保护，
  因此不计为精度通过。之后已按用户确认把规则改为达到 60°C 停止。

上述结果只证明当时起点和路径可执行，不代表任意起点、任意方向、
任意负载都已通过。

## 15. 已知剩余风险

- 尚未完成全局 XYZ 工作空间和全部关节的实机绝对边界认证。
- 坐标轴尚未在实际桌面工位进行永久标记。
- `move_to()` 只保持当前姿态，尚不支持显式 roll/pitch/yaw。
- 没有碰撞模型、视觉避障、负载估计或独立硬件急停接口。
- 初级教学版机械臂有回差、重力下垂和精度波动，6 mm 是本轮使用的
  运行验证容差，不是厂商精度声明。
- 真实摄像头已识别并完成软件接入，但尚未抓取单帧。
- `main.py` 尚未提供真机 Gateway 启动方式，默认只能使用 Mock。
- 早期连接、夹爪和 stop 手动脚本会在退出时明确警告
  `disconnect()` 不是关力矩；正式 `move_to` 脚本已执行完整的关力矩清理。
- 摄像头与机械臂组合后的真实连接顺序尚未进行实机验证。

## 16. 代码位置

| 作用 | 文件 |
| --- | --- |
| 统一 Adapter 接口 | `src/rosclaw_mini/arm/base.py` |
| SO-100 Plus 真实 Adapter | `src/rosclaw_mini/arm/so100_plus.py` |
| 运动学和 TCP | `src/rosclaw_mini/arm/kinematics.py` |
| 机器、校准和摄像头 factory | `src/rosclaw_mini/arm/so100_plus_factory.py` |
| 工作空间和关节限制 | `src/rosclaw_mini/safety/limits.py` |
| Skill 到 Adapter 映射 | `src/rosclaw_mini/skills/arm_handler.py` |
| Skill 参数范围 | `src/rosclaw_mini/skills/arm_skills.py` |
| 命令通用安全检查 | `src/rosclaw_mini/safety/checker.py` |
| 真机手动检查 | `scripts/check_so100_plus_*.py` |
