# RosClaw Mini 机械臂动作与 SO-100 Plus 真机接入

本文档记录当前已实现的机械臂命令链路、SO-100 Plus
right follower 的实际配置、坐标语义、安全限制和手动验证方法。

> 默认应用入口仍使用 `MockArmAdapter`。`pytest` 和不带真机参数的
> `main.py` 不会连接硬件；只有显式选择 `--backend so100_plus` 并传入
> `--acknowledge-so100-plus-risk` 后，统一入口才会装配并连接真机。

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

`main.py` 已提供 `mock/so100_plus` 后端选择，默认仍是 `mock`。
本文后面的真机脚本保留为单项诊断和人工验证工具，不会被默认业务入口
或单元测试自动调用。

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
| 肘关节 P | 连接后 64 |
| 腕部俯仰 P | 连接后 24 |

代码不随机选择 `main_follower.json` 或 `right_follower.json`。
`SO100PlusRobotConfig` 根据显式的 `follower_name` 构造文件名；
统一入口还把正式工作空间绑定到已认证 `right_follower.json` 的 SHA-256
`ac7b9877020da10aa6f886347bedf6b105aaeaf01493b2a65830c628c35837de`；
认证端口固定为 `/dev/lerobot_right`。目录可以改变，但其他端口或文件
内容变化后不能继续套用该工作空间。
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

这里必须区分两个容易混淆的姿态：

- `follower_rest`：README 校准照片里的折叠收纳姿态；当前真机上电前
  就是这个姿态；
- `SO100_PLUS_JOYCON_INITIAL_RADIANS`：`JoyConController_plus.init_qpos`
  定义的控制器初始工作姿态，关节角为
  `(0, -3.1, 3.0, 0.0, 0.0, 1.57)` rad。

此前文档把后者称为 “JoyCon rest”，这是不准确的；仿真候选框实际
围绕控制器初始工作姿态建立。

`follower_rest` 是人工摆放的收纳外形，不要求每次精确复现一个关节
数组。当前脚本对肩俯仰使用 8°识别余量、其他主要折叠关节使用 5°，
腕偏航/滚转使用 20°/15°；随后必须以当次实测六关节角重新通过
MuJoCo 单调上升、无新增接触检查，不能只靠角度余量放行。

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

### 4.6 `move_joints(joint_radians)`

`SO100PlusAdapter.move_joints()` 接收六个模型关节角，单位为弧度，
顺序与前六个电机一致。它用于 `move_to()` 无法完成的姿态变化，例如
从折叠收纳姿态展开到 JoyCon 初始工作姿态。

这个动作不会绕过原有保护：目标关节、目标 TCP 和插值关节步长都必须
通过显式 `MotionLimits`，执行继续使用 30 Hz 平滑轨迹、跟踪误差、
负载、温度和最终 TCP 检查，并保持第七个夹爪电机位置不变。它目前
只在显式真机脚本中使用，尚未映射到 Gateway/Skill。

当前这台教学版 `right_follower` 在首次收纳姿态展开时，腕俯仰等待
8 秒后仍有 `2.162°` 稳态误差，但负载、温度和动作均正常。过渡和
边界验收因此使用 `3°` 最终关节门槛和 `12 mm` 最终 TCP 容差。
完成边界代表点测试并登记正式工作空间后，用户确认把同一组
`3° / 12 mm` 保存为普通 `move_to()` 的正式运行默认值。
进入每个候选框目标前，脚本还会从当次实测关节姿态重新按 1°检查
MuJoCo 接触和 TCP 的 Z=0 支撑平面，而不是直接假定姿态完全等于仿真值。

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

### 5.4 仿真候选工作空间

已经使用 MuJoCo 对 1,000,000 个六关节姿态完成离线扫描。底座保持
`right_follower` 实测旋转范围，其他五个关节使用 MuJoCo 模型范围，
并过滤模型自碰撞、地面接触和 TCP 低于地面的样本。

527,612 个样本成为无碰撞候选。点云独立轴向外边界约为：

```text
X: -0.342855 .. 0.530123 m
Y: -0.311638 .. 0.276831 m
Z:  0.000006 .. 0.553846 m
```

这个外包围盒不是完整可达长方体，因此没有写入 `WorkspaceLimits`。
仿真方法、点云图片、碰撞统计和复现命令见
[SO-100 Plus 仿真候选工作空间](so100_plus_simulated_workspace.md)。

### 5.5 JoyCon 初始工作姿态候选框

在所有姿态点云之后，又固定 JoyCon 控制器初始 TCP 姿态执行了
1 cm 笛卡尔网格精扫。每个点都检查“初始姿态→目标”的 1° 关节
插值路径。

最大全通过候选框是：

```text
X: 0.303571 .. 0.443571 m
Y: -0.051185 .. 0.028815 m
Z: 0.169328 .. 0.309328 m
```

框内 2,025 个网格点全部通过；11,160 条相邻网格有向边也全部通过
重新 IK 和路径碰撞检查。初始工作点位于 X 下边界，因为肩、肘关节
接近模型端点。

这个外层范围仍然是仿真候选。真机代表点测试完成后，正式
`WorkspaceLimits` 采用每个面内缩 `1 cm` 的范围：

```text
X:  0.313571423 .. 0.433571423 m
Y: -0.041185494 .. 0.018814506 m
Z:  0.179328483 .. 0.299328483 m
```

代码常量为 `SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS`。完整仿真方法
和限制见
[SO-100 Plus 仿真候选工作空间](so100_plus_simulated_workspace.md#10-joycon-初始工作姿态精扫)。

真机脚本 `scripts/check_so100_plus_candidate_workspace.py` 默认验证内部
代表点；增加 `--transition-only` 后，只执行
`follower_rest → storage_escape → JoyCon 初始姿态 → storage_escape
→ follower_rest`，不会生成或执行候选框内部检查点。两种模式共用相同
的只读预检、MuJoCo 路径检查、遥测保护、Rest 到位验证和力矩释放规则。

增加 `--boundary-suite` 后，脚本会在同一次运行中先重复 4 个内部往返
检查点，再验证距候选框六个面和八个角各一个 `1 cm` 仿真网格步长的
14 个代表点，最后回到 JoyCon 初始姿态；合计 19 个候选区检查点。
顺序经过连续 IK 和逐段 MuJoCo 检查，且 `--transition-only` 与
`--boundary-suite` 互斥。这个套件是边界代表点实机验收，不等同于把
2,025 个仿真网格点全部逐个搬到真机执行。

首次完整边界套件已经通过前九个候选区检查点；第十个
`boundary_face_x_max` 得到明确的到位误差失败结果。第一次
`--boundary-resume` 又通过
`boundary_corner_x_max_y_min_z_max`，随后
`boundary_corner_x_max_y_max_z_max` 以 `14.780 mm` TCP 误差超过
`12 mm` 门槛。自动收纳时肩俯仰的瞬时跟踪误差为 `5.006°`，
超过 `5°` 门槛 `0.006°`，因此程序紧急关闭力矩。

第二次续测已经执行剩余六个边界代表点，TCP 误差依次为
`2.158、1.142、3.635、4.995、3.832、9.914 mm`，全部低于
`12 mm` 验收门槛；返回 JoyCon 初始姿态误差为 `2.644 mm`。
最终受控回到 `follower_rest` 并正常关闭力矩。该轮最高负载 `285`、
最高电流 `43`、最高温度单次读数 `57°C`，没有触发保护。

至此 14 个边界代表点均已有真机结果：12 个通过，2 个因到位精度失败。
失败点是 `boundary_face_x_max`（约 `24.800 mm`）和
`boundary_corner_x_max_y_max_z_max`（`14.780 mm`）。因此代表点测试
已经完成，但整个长方体仍不能作为满足 `12 mm` 精度的正式工作空间。
当前 `--boundary-resume` 会在访问串口前拒绝重复执行。

与 `--continue-on-convergence-error` 一起使用时，候选点轨迹安全完成、但
最终关节或 TCP 到位误差超过验收门槛，只会把该点记录为失败，然后从
当次实测姿态重新规划下一个点。全部点执行完后再统一收纳和汇总，进程
仍返回非零表示候选框没有整体通过。这个选项不会忽略路径碰撞、关节
越界、过载、过温、运动中跟踪误差或通信异常；这些情况仍会立即停止。

## 6. 已保存的真机运行配置

`SO100PlusRealHardwareProfile` 保存了上一轮真机验证使用的参数：

| 配置 | 数值 | 语义 |
| --- | ---: | --- |
| 其他电机 P | 16 | 连接时显式写入 |
| 肘关节 P | 64 | 负载下改善跟踪 |
| 腕部俯仰 P | 24 | 减少腕部稳态误差 |
| 运行时加速度 | 35 | 只写 RAM，不写 EEPROM/Lock |
| 夹爪单步 | 10° | 分步打开/关闭 |
| 夹爪步间等待 | 2.5 s | 每步后等待反馈 |
| 夹爪负载上限 | 300 | 夹持堵转保护 |
| 夹爪位置容差 | 3° | 目标与实测差值 |
| 轨迹最终关节容差 | 3.0° | 六关节全部需满足 |
| 最终 TCP 容差 | 12 mm | FK 复算目标 |
| 手臂普通过载上限 | 450 | 同一电机连续两次达到时停止 |
| 手臂紧急负载上限 | 700 | 单次达到时立即停止 |
| 普通过温上限 | 60°C | 同一电机连续两次达到时停止 |
| 紧急温度上限 | 70°C | 单次达到时立即停止 |
| 流式频率 | 30 Hz | 余弦缓入缓出目标 |
| 最大关节速度 | 20°/s | 流式曲线峰值限制 |
| 流式跟踪误差上限 | 5° | 超限立即保持实测位置 |
| 遥测间隔 | 0.25 s | 记录电压、电流、负载、温度 |
| 最终到位超时 | 8 s | 超时后保持当前位置 |
| 最终稳定观察 | 0.75 s | 进入 3°后连续观察，避免过早检查 TCP |

稳定观察会记录六个关节的全部位置样本和各关节峰峰值，并用每个样本
复算 TCP。真机 JSONL 日志保存 TCP 的逐样本位置、XYZ 最小值、最大值
和平均值；最终 12 mm 检查使用稳定窗口的最新位置。30 Hz 只缩小运动中
相邻目标的间隔，总速度仍为 20°/s，不会跳过关节、工作空间或遥测保护。

腕部俯仰 P=24 的首次真机验证中，连接后的写入和回读成功，
`storage_escape` 与 JoyCon 初始工作姿态分别以 `10.286 mm` 和
`11.580 mm` TCP 误差通过过渡门槛。进入第一个候选框目标后，
`wrist_pitch_joint` 最终误差为 `1.547°`，比当时的候选区门槛
`1.5°` 多 `0.047°`，因此脚本按设计保持当前位置并关闭全部力矩。
本次最高温度 `39°C`、最高负载 `272/300`。P=24 保留；考虑到这是
教学版机械臂，用户当时确认将关节容差调整为 `2.0°`，TCP 容差仍
保持 `6 mm`；这是完成边界验收前的历史配置。

调整为 `2.0°` 后的下一次真机验证中，`storage_escape` 和 JoyCon
初始工作姿态分别以 `8.980 mm`、`10.352 mm` TCP 误差通过过渡门槛；
第一个候选点不再触发关节误差保护，但最终 TCP 误差为
`10.755 mm`，超过候选区正式门槛 `6 mm`，因此再次安全停止并关闭
全部力矩。本次最高温度 `40°C`、最高负载 `256/300`。这说明当前
主要限制来自多个关节各自小于 2°、但可以重复出现的稳态滞后，它们
组合后形成更大的 TCP 误差；候选框仍未完成整套实机认证。

改为 30 Hz 并增加 0.75 秒稳定观察后的真机记录进一步排除了“检测
太早”和“持续抖动”两个假设。`storage_escape` 的最大关节峰峰值为
`0.352°`，TCP 单轴最大波动为 `1.061 mm`；JoyCon 初始工作姿态和
第一个候选点的六关节峰峰值均为 `0.000°`，但稳定 TCP 误差仍分别
为 `10.384 mm` 和 `11.907 mm`。第一个候选点主要表现为 Z 方向低
`11.571 mm`；底座、肘和腕俯仰分别稳定滞后约 `0.769°`、
`1.099°` 和 `1.677°`。本次最高温度 `39°C`、最高负载
`256/300`，脚本因
6 mm TCP 门槛正常停止并关闭力矩。由此可知，约 1.2 cm 的误差不是
轻微抖动造成的。

### 6.1 有限轮自动 PID 调参

`scripts/tune_so100_plus_pid.py` 用固定候选代替人工反复改参数。P 保持
当前实机值（底座 16、肘 64、腕俯仰 24），最多测试：

```text
I0/D0 → I0/D16 → I0/D32
→ I1/前三组最佳 D → I2/前三组最佳 D
```

每组只有一个 `JoyCon 初始姿态 ↔ near_internal` 往返；达到
`TCP 误差 <= 6 mm` 且 `TCP 单轴稳定波动 <= 2 mm` 会提前停止。
五组仍未达到时，脚本选择评分最佳组。如果最佳组的六关节实测残差
都不超过 2°，最多再执行一次反向残差补偿；没有改善就停止，不会再
增加第七次、第八次试验。

P/I/D 是 EEPROM 寄存器。`SO100PlusAdapter.set_pid_gains()` 因此要求
显式 `acknowledge_eprom_write=True`，写入时会：

```text
保持全部关节当前位置
→ 读取并保存原 PID
→ 只解锁需要变化的电机
→ 只写发生变化的 P/I/D
→ 立即重新上锁
→ 读回核对
```

脚本最多五组 PID、五个验证往返，加一次残差补偿往返；展开和收纳另算。
它不动作夹爪。原有 5° 跟踪误差、夹爪 300 负载、手臂
450 连续/700 紧急负载、60°C 连续过温确认、70°C 紧急温度、
工作空间、关节范围和 MuJoCo 路径检查仍全部生效。异常时恢复连接后
的 PID 基线，保持当前位置、关闭力矩并关闭串口。

结果写入 JSONL，脚本本身不会自动改源码。最终固定同一个六关节目标
进行 A/B：`I=2/D=16` 的 TCP 误差为 `8.583 mm`、单轴稳定波动为
`3.652 mm`；`I=2/D=32` 的 TCP 误差为 `7.396 mm`、单轴稳定波动为
`0.291 mm`。两组温度均为 `39°C`，最高负载分别为 260 和 267。

因此正式实机配置选择 `I=2/D=32`，应用于底座旋转、肘和腕俯仰；
三者 P 分别为 16、64、24。其余手臂关节仍为 `P=16/I=0/D=0`。
固定目标误差仍超过 6 mm，因此这里只确认 D32 是本次更好的配置，
不把候选点标记为 6 mm 精度通过。

旧版 LeRobot 预设会在连接时写回 `P=16/I=0/D=0` 并打开 EEPROM
写锁。Adapter 在其后先保持实测位置、统一上锁，只恢复数值不同的
正式 PID，然后逐个上锁并读回核对。

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
| `disable_torque()` | 验证 follower_rest 后写 `Torque_Enable = 0`，确认全部关闭，并保持 EEPROM 写锁 | 关闭 | 正常收纳后机械臂变软 |
| `disconnect()` | 关闭 LeRobot/串口通信 | 不保证关闭 | 只表示无法继续发命令，不是停止或急停 |

需要主动收纳并卸力的真机动作脚本，安全退出顺序应为：

```text
adapter.stop()
→ 受控返回并验证 follower_rest
→ adapter.disable_torque()
→ adapter.disconnect()
```

普通 `disable_torque()` 在当前位置不满足本机 `follower_rest` 容差时
会拒绝释放。只有过温、过载、碰撞、人工急停或受控收纳失败时，调用方
才可显式使用 `disable_torque(emergency=True)`；这条路径会记录
`torque_disabled_emergency` 遥测，并要求人员托住机械臂。不能把
`disconnect()` 当成关力矩，软件也不替代物理断电。

统一 JSON 入口遵循另一条明确约束：普通退出或 Ctrl+C 只执行
`stop() → 最多等待后台动作 5 秒 → disconnect()`，不自动调用
`disable_torque()`。即使 `stop()` 报错，也会继续等待 Controller；后台
线程超时时不会立即断开，也不报告安全完成；非 daemon 延后清理线程会
继续以有界等待轮询，并在线程稍后结束后完成 `disconnect()`。是否收纳
和卸力必须由操作者另行决定。

### 当前 `stop` 的系统限制

`SO100PlusAdapter.stop()` 已能设置停止事件、读取当前位置并把当前位置
写回目标位置，因此 Adapter 层的取消能力已经存在。

当前 `main.py` 使用 `ExecutionController` 在后台执行普通命令：

```text
读取 move_arm
→ ExecutionController 在后台调用 run_command()
→ 主输入循环继续接收 stop
→ controller.request_stop()
→ SO100PlusAdapter.stop()
```

因此，在同一个命令行输入循环中，可以在 `move_to()` 执行期间输入
`stop` 来中断 Adapter 轨迹。一次仍只允许一个普通命令运行；新的普通
命令必须等待前一个完成或先停止前一个。

这表示：

- Adapter 层停止能力：已实现；
- 命令行入口的运动中 stop：已接入；
- `stop()` 不是独立硬件急停，也不能替代物理断电。

## 9. 连接时会发生什么

`ManipulatorRobot.connect()` 不是只读操作。它会：

1. 打开 follower 串口；
2. 加载已有校准；
3. 暂时切换力矩和写入运行配置；
4. 最后启用 follower 力矩。

`SO100PlusAdapter.connect()` 在此基础上还会：

1. 先把 `Goal_Position` 同步到实测位置，避免旧目标导致跳动；
2. 关闭旧预设留下的 EEPROM 解锁状态；
3. 恢复并回读正式 PID：底座旋转 `16/2/32`、肘 `64/2/32`、
   腕部俯仰 `24/2/32`，其余手臂关节 `16/0/0`；
4. 只写入与当前寄存器不同的 PID，并在每个电机写完后立即上锁；
5. 写入运行时加速度 35；
6. 读取电压、电流、负载和温度。

摄像头不是机械臂连接前置条件。`connect()` 和 `disconnect()` 只管理
机械臂，不读取、连接或关闭摄像头。

## 10. 摄像头

已实现 `SO100PlusCameraConfig`、`create_so100_plus_cameras()`、
`SO100PlusAdapter.connect_cameras()`、`disconnect_cameras()` 和
`capture_camera_images()`。

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

摄像头按需独立使用：

```python
adapter.connect_cameras()
images = adapter.capture_camera_images()
adapter.disconnect_cameras()
```

这些调用不要求机械臂已连接，也不会改变机械臂、力矩或串口状态。
反过来，机械臂 `connect()`/`disconnect()` 也不改变摄像头状态。
摄像头连接失败只回滚本次新连接的摄像头。

## 11. 技能和默认后端

当前 `build_arm_skills()` 支持：

| Skill | Adapter 原子动作 | 状态 |
| --- | --- | --- |
| `move_arm` | `move_to(x, y, z)` | 通用构造函数需显式范围；right follower 可使用已登记的专用构造函数 |
| `open_gripper` | `open_gripper()` | 已实现 |
| `close_gripper` | `close_gripper()` | 已实现 |
| `stop` | `stop()` | 已实现 |
| `disable_torque` | `disable_torque()` | 底层已实现；高风险，默认不向 Gateway/LLM 开放 |

`disable_torque()` 仍供本地维护和受控关闭流程直接调用；默认禁用的是
普通 Gateway/LLM Skill 入口，不是删除底层卸力能力。

`main.py` 默认创建 `MockArmAdapter`。显式选择 `--backend so100_plus`
并确认风险时，`runtime.py` 会复用现有 Robot Factory、运动学、正式
MotionLimits 和 right-follower Skills，把真实 Adapter 接入相同 Gateway
与 ExecutionController。摄像头不参与这次装配。

真机连接后，`runtime.py` 会用当前六关节位置计算启动 TCP，并同时检查
关节姿态是否满足原真机验收脚本
`MAX_INITIAL_JOINT_ERROR_DEGREES = 5.0` 采用的启动门槛。运行时直接使用
`abs(actual - expected)`，不把 `expected ± 2π` 折叠为等价角。这个数值
是验收采用的启动判定门槛，不表示六关节任意独立 `±5°` 组合都经过真机
验证。TCP 或关节姿态任一不符合时，只把运行时 Skill 注册表里的
`move_arm.enabled` 复制为 `False`，不会改写通用 Skill 定义、Safety
Checker、运动学或 Adapter；夹爪和 `stop` 仍保持启用。Gateway 因而会在调用
`SO100PlusAdapter.move_to()` 之前返回 `技能未启用: move_arm`。这用于
防止机械臂仍在 `follower_rest` 时直接求解工作区目标；当前没有把未经
统一入口验收的收纳展开轨迹伪装成普通 `move_arm`。门禁不会在同一进程
中自动解除；使用单独验收的流程进入工作区后，应重新启动统一入口。

`build_so100_plus_right_follower_arm_skills(adapter)` 会把
`SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS` 写入 `move_arm` 的
`x/y/z` 参数边界。Adapter 侧应使用
`build_so100_plus_right_follower_motion_limits(current_joint_radians)`，
让 Gateway 检查和底层运动规划共享同一正式范围。

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

### 13.6 有限轮 PID 自动调参

开始前必须关闭力矩，把机械臂人工放回本机 `follower_rest` 收纳姿态。
先运行只读预检；它会读位置并做 MuJoCo 展开路径检查，但不会写 PID
或发送运动目标：

```bash
python scripts/tune_so100_plus_pid.py \
  --port /dev/lerobot_right \
  --calibration-dir lerobot-joycon_plus/.cache/calibration/so100_plus \
  --follower-name right \
  --preflight-only
```

只有只读预检通过、路径清空、操作者能够托住机械臂并立即物理断电时，
才可执行硬件模式：

```bash
python scripts/tune_so100_plus_pid.py \
  --port /dev/lerobot_right \
  --calibration-dir lerobot-joycon_plus/.cache/calibration/so100_plus \
  --follower-name right \
  --acknowledge-bounded-pid-eprom-tuning-risk
```

硬件模式会展开和收纳机械臂，最多进行五个目标往返和一个补偿往返，
并对三个电机的 PID EEPROM 做有限次写入。完成或失败后都会停止轨迹、
关闭力矩并断开串口；失败时机械臂可能停在当时位置，不保证自动收纳。

## 14. 已执行的真机验证

本轮已完成：

- 确认型号为 SO-100 Plus，follower 为 `right`；
- 确认 `/dev/lerobot_right` 对应实际 follower 串口；
- 读取 7 个电机的真实位置；
- 夹爪开合和 `stop()` 保持位置；
- 力矩关闭和底座安全角度手动选取；
- 固定相同关节目标比较 I2/D16 与 I2/D32，保存 I2/D32；
- 多次局部 `move_to` 验证；
- 分段 +Z 10 cm：实际累计约 9.869 cm，最终 TCP 误差约 1.742 mm；
- 单次笛卡尔计划 +Z 10 cm：实际累计约 9.743 cm，最终 TCP 误差约 2.804 mm；
- 20 Hz 流式 +Z 10 cm：实际累计约 10.144 cm；此次最终 TCP
  误差约 8.378 mm，当时因 7 号电机 44°C 相对温升规则触发旧保护，
  因此不计为精度通过。当前规则为同一电机连续两次达到 60°C 停止；
  单次达到 70°C 则立即停止。

上述结果只证明当时起点和路径可执行，不代表任意起点、任意方向、
任意负载都已通过。

## 15. 已知剩余风险

- 尚未完成全局 XYZ 工作空间和全部关节的实机绝对边界认证。
- 坐标轴尚未在实际桌面工位进行永久标记。
- `move_to()` 只保持当前姿态，尚不支持显式 roll/pitch/yaw。
- 没有碰撞模型、视觉避障、负载估计或独立硬件急停接口。
- 初级教学版机械臂有回差、重力下垂和精度波动，当前 `12 mm` 是项目
  运行验收容差，不是厂商精度声明。
- 真实摄像头已识别并完成软件接入，但尚未抓取单帧。
- `main.py` 的真机连接、普通退出、Ctrl+C、`stop`、夹爪动作和不自动
  卸力已经由操作者完成实机检查；从 `follower_rest` 启动时，当前 TCP
  位于正式工作空间外，因此普通 `move_arm` 会失败关闭。尚未实现并验收
  统一入口专用的安全展开/收纳转换动作；默认仍是 Mock。
- 早期连接、夹爪和 stop 手动脚本会在退出时明确警告
  `disconnect()` 不是关力矩；正式 `move_to` 脚本已执行完整的关力矩清理。
- 摄像头已经与机械臂生命周期解耦，但真实摄像头单帧仍未验证。

## 16. 代码位置

| 作用 | 文件 |
| --- | --- |
| 统一 Adapter 接口 | `src/rosclaw_mini/arm/base.py` |
| 应用 Adapter、Skills 和 Controller 装配 | `src/rosclaw_mini/runtime.py` |
| SO-100 Plus 真实 Adapter | `src/rosclaw_mini/arm/so100_plus.py` |
| 运动学和 TCP | `src/rosclaw_mini/arm/kinematics.py` |
| 机器、校准和摄像头 factory | `src/rosclaw_mini/arm/so100_plus_factory.py` |
| 工作空间和关节限制 | `src/rosclaw_mini/safety/limits.py` |
| Skill 到 Adapter 映射 | `src/rosclaw_mini/skills/arm_handler.py` |
| Skill 参数范围 | `src/rosclaw_mini/skills/arm_skills.py` |
| 命令通用安全检查 | `src/rosclaw_mini/safety/checker.py` |
| 真机手动检查 | `scripts/check_so100_plus_*.py` |
