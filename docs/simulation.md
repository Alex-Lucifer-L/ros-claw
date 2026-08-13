# Headless SO-100 Plus 抓取仿真

## 目的与边界

`--backend sim` 是独立的、本地的研究后端，用于比较提示词和抓取策略。
它加载仓库已有 SO-100 Plus MuJoCo 模型、运动学和不规则工作空间，但绝不
打开串口、`/dev`、真实 RealSense pipeline 或网络 API。它不会自动切换到
`so100_plus`，即使电脑上存在设备。

安装可选依赖：

```bash
/home/alex/miniconda3/envs/rosclaw-mini-py310/bin/python -m pip install -r requirements-sim.txt
```

当前环境已经可用 MuJoCo 时不需要重复安装。

## 使用

所有命令均为 headless、仿真-only：

```bash
MPLCONFIGDIR=/tmp PYTHONPATH=src:lerobot-joycon_plus \
  python -m rosclaw_mini.main --backend sim --input-mode json

# 可显式读取仿真相机文件；缺少 simulation_only=true 的真实标定会被拒绝
MPLCONFIGDIR=/tmp PYTHONPATH=src:lerobot-joycon_plus \
  python -m rosclaw_mini.main --backend sim --sim-camera-config \
  configs/simulation_camera.example.json

MPLCONFIGDIR=/tmp PYTHONPATH=src:lerobot-joycon_plus \
  python scripts/run_sim_grasp.py --scene standard --strategy baseline_v1

MPLCONFIGDIR=/tmp PYTHONPATH=src:lerobot-joycon_plus \
  python scripts/run_sim_grasp.py --scene noisy --strategy humanlike_v3

MPLCONFIGDIR=/tmp PYTHONPATH=src:lerobot-joycon_plus \
  python scripts/run_sim_grasp_benchmark.py --seeds 3 \
  --output artifacts/simulation/benchmark.json
```

`--backend sim --sim-start-state rest` 可验证 `REST → unfold_arm → WORK`
软件会话门禁；默认 `work`，以便直接批量运行抓取实验。模拟 REST/WORK
状态不等价于真机 follower_rest 认证。

## 数据流

```text
自然语言任务
  → FakeGraspStrategyLLM（只给高层策略）
  → VirtualRGBDCamera 同步 RGB/Depth
  → SimulatedColorVLM（只读取图像像素，不读取物体真值）
  → 现有 bounding box / 稳健深度 / 反投影
  → simulation_only eye-to-hand 变换
  → 现有 GraspPlan / Gateway / Safety Checker
  → ExecutionController
  → SimulatedArmAdapter / MuJoCo 轨迹预检
  → 虚拟夹爪保持与 lift 验证
```

仿真器内部真值只用于渲染与 benchmark 误差/成功率统计。普通规划器不接收
真值坐标。`ground_truth` 比较可在单元测试中使用，不能作为正常实验输入。

## 场景、策略和评价

场景：`standard`、`multi_object`、`randomized`、`noisy`、`depth_holes`、
`boundary_reject`。每一场景可用固定随机种子重现；`noisy` 同时注入少量
RGB/Depth 噪声、相机位姿扰动和图像遮挡，`depth_holes` 用于深度质量门禁。
默认 benchmark 在 `multi_object` 场景按种子轮换蓝色盒子和绿色圆柱体，因而
以相同的场景/种子比较不同策略时不会把物体形状差异混入策略差异。

提示词对照文件位于 `prompts/grasp_baseline_v1.md`、
`prompts/grasp_efficient_v2.md`、`prompts/grasp_humanlike_v3.md`。它们只
允许输出目标、抓取风格、接近方向、偏航偏好和是否重新观察；确定性代码
仍负责定位、IK、轨迹、安全检查和执行。

benchmark 记录成功率、运动时间、TCP 路径、关节总运动、等价平滑度、定位
误差、重新观察次数、碰撞事件和工作空间拒绝。输出 JSON 和 Markdown 为
真实执行结果，且明确标注 simulation-only。

`docs/simulation_fidelity.md` 列出可复用资产、估计参数和回实验室后的
sim-to-real 验收，`docs/simulation_blockers.md` 记录当前未解决的现场依赖。
