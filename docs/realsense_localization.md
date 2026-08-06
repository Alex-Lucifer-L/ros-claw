# RealSense RGBD 目标定位与固定相机外参

本模块为现有 Webcam 只读观察链增加一条**可选、独立**的 RealSense
RGBD 链。默认入口、Mock、SO-100 Plus、原 Webcam 和现有腕部相机
eye-in-hand 标定都不会因此改变。

当前安全边界：

```text
RealSense RGB + 对齐 Depth
→ 千问只选择一个目标并返回原图整数像素框
→ 程序严格验证并归一化 bounding_box
→ 中央 ROI 稳健深度
→ Color 光学坐标三维点
→ （只有真实 eye-to-hand 标定激活后）机械臂基座坐标
```

这条链当前只读，不创建 `ArmRuntime`、`ExecutionController` 或运动
`Command`。VLM 只选择目标和边界框；深度与三维坐标来自同一组 RealSense
帧和运行时内参。

## 可选依赖

```bash
python -m pip install -r requirements-realsense.txt
```

`pyrealsense2` 只在真正打开 RealSense 时加载。默认 pytest 使用 Fake SDK，
不需要相机。设备序列号保存在环境变量或 `local_calibration/` 中，不写入
通用源码：

```bash
export ROSCLAW_REALSENSE_SERIAL="<设备序列号>"
```

## 只读 RGBD 诊断

```bash
PYTHONPATH=src python scripts/check_realsense_camera.py \
  --serial "$ROSCLAW_REALSENSE_SERIAL" \
  --width 640 --height 480 --fps 30 --warmup-frames 10 --frames 30
```

Adapter 使用序列号选择设备，不依赖 `/dev/videoN`。它同步读取 RGB8 和
Z16，执行 `align(rs.stream.color)`，并从实际 profile 读取 Color 内参和
depth scale。pipeline 在成功、异常、超时和上下文退出时都会关闭。

## 只读目标相机坐标

配置百炼 Key 后运行：

```bash
PYTHONPATH=src python scripts/locate_realsense_object.py \
  --serial "$ROSCLAW_REALSENSE_SERIAL" \
  --question "定位桌面上的红色盒子"
```

输出 `PositionEstimate`：

- `bounding_box`：程序将 VLM 的原图整数像素框严格检查后归一化的框；
- `center_pixel`：框中心 Color 像素；
- `depth_m`：框中央收缩 ROI 的稳健中位深度；
- `camera_point_m`：Color 光学坐标，`+X` 向右、`+Y` 向下、`+Z` 向前；
- `valid_depth_ratio/uncertainty_m/quality`：深度质量；
- `source_frame/source_timestamp_ms`：与观察绑定的源帧。

程序拒绝非法/空/过小框、低有效深度率、NaN/Inf、离群点过滤后样本不足、
深度双表面离散过大和多目标歧义。它不会读取中心单像素，也不会用
left/center/right 语义生成三维坐标。

定位专用 Prompt 会把实际 JPEG 宽高告诉模型，并要求
`bounding_box_pixels=[x_min,y_min,x_max,y_max]` 整数原图像素。
不接受模型直接生成的 0–1 语义估计框；归一化只由本地代码
在边界和整数类型验证通过后执行。普通 Webcam 场景观察协议不受影响。

## 固定相机 eye-to-hand 数据

固定 D435i 需要求：

```text
P_base = T_base_from_camera × P_camera
```

这与腕部相机已有的 `tcp_T_camera` 不同，二者不能混用。每个真实对应点
需要同一个物理标记点的：

- `camera_point_m`：RealSense 定位结果；
- `base_point_m`：该物理点在 SO-100 Plus 基座坐标系中的独立已知值。

记录一个点对（命令本身不访问硬件）：

```bash
PYTHONPATH=src python scripts/record_realsense_eye_to_hand_point.py \
  --dataset local_calibration/realsense_d435i/eye_to_hand/point_pairs.json \
  --serial "$ROSCLAW_REALSENSE_SERIAL" \
  --camera-point <CX> <CY> <CZ> \
  --base-point <BX> <BY> <BZ> \
  --split fit
```

如果已将同一个物理标记点与当前机械臂 TCP 对齐，可用单次
只读采集命令避免手工抄写两组坐标：

```bash
PYTHONPATH=src:lerobot-joycon_plus python \
  scripts/capture_realsense_eye_to_hand_point.py \
  --dataset local_calibration/realsense_d435i/eye_to_hand/point_pairs.json \
  --serial "$ROSCLAW_REALSENSE_SERIAL" \
  --port /dev/lerobot_right \
  --calibration-dir lerobot-joycon_plus/.cache/calibration/so100_plus \
  --follower-name right \
  --question "只定位与机械臂 TCP 物理对齐的红色标定点" \
  --split fit \
  --acknowledge-readonly-arm-capture \
  --acknowledge-camera-cloud-upload
```

该命令要求所有电机力矩已关闭，先读关节反馈，采集并上传一张
D435i RGB 帧完成目标定位，再读一次关节反馈。采集前后任一
关节漂移超过 `1°` 就拒绝保存。它只读打开 follower 串口，不写
`Goal_Position`、力矩、PID 或校准寄存器，不会自行移动机械臂。
因为它不产生运动，手动教学姿态只检查真实反馈数量和有限性，
不用 WORK 运动关节范围拒绝只读点对。这不会改变或放宽任何正式
`move_arm` / `move_relative` 运动门禁。

“物理对齐”是标定成立的前提：D435i 定位的必须是与模型 TCP
同一个空间点。如果看到的是标记物中心，而记录的是夹爪其他位置，
会产生固定偏移，即使数学残差很小也不能作为真实外参。

至少六个拟合点，推荐采集八至十二个，覆盖工作区不同 X/Y/Z，不能共线；
另留至少三个 `--split validation` 点用于独立验收。标定目标、相机和机械臂
底座在整个数据集期间都不能移动。

离线求解：

```bash
PYTHONPATH=src python scripts/calibrate_realsense_eye_to_hand.py \
  --dataset local_calibration/realsense_d435i/eye_to_hand/point_pairs.json \
  --output local_calibration/realsense_d435i/eye_to_hand/base_T_camera.json \
  --max-rmse-mm 20 \
  --max-error-mm 40
```

求解器使用SVD刚体拟合，检查点分布、旋转正交性和 `det(R)=1`，输出逐点
残差、训练/留出RMSE和最大误差。超过阈值的文件会写成 `active=false`，
正式加载和基座坐标转换仍会拒绝。文件绑定相机序列号、640×480分辨率、
坐标系、单位和SHA-256。

`configs/realsense_eye_to_hand.example.json` 只说明结构，故意没有矩阵，不能
作为正式标定使用。所有真实点对、矩阵和现场图片都应保存在已被
`.gitignore` 忽略的 `local_calibration/`。

外参激活后，可在不创建机械臂 Runtime、不生成运动命令的情况下，
将当次 RealSense 定位结果转换为基座坐标：

```bash
PYTHONPATH=src python scripts/locate_realsense_object.py \
  --serial "$ROSCLAW_REALSENSE_SERIAL" \
  --question "只定位画面中的红色瓶盖" \
  --eye-to-hand-calibration \
    local_calibration/realsense_d435i/eye_to_hand/base_T_camera.json
```

输出保留原始 `position_estimate.camera_point_m`，并增加
`base_position_estimate.base_point_m`、标定哈希、拟合/验证误差和当次定位
质量。标定未激活、序列号不匹配或分辨率不匹配时失败关闭。

## 抓取计划预览（不运动）

下面的命令将定位结果生成固定的
`open_gripper → pre_grasp → approach → close_gripper → lift`
计划，对每一步调用现有 Gateway/Safety Checker，并检查 SO-100 Plus
正式不规则工作空间。它只输出预览，不创建 Runtime：

```bash
PYTHONPATH=src python scripts/preview_realsense_grasp_plan.py \
  --serial "$ROSCLAW_REALSENSE_SERIAL" \
  --question "抓取画面中的红色物体" \
  --eye-to-hand-calibration \
    local_calibration/realsense_d435i/eye_to_hand/base_T_camera.json \
  --acknowledge-camera-cloud-upload
```

目标或 pre-grasp/lift 中任一点不在正式网格、数据过期、深度不可靠、
外参未激活或 Safety Checker 拒绝时，预览返回非零退出码。

## 尚未开放的能力

在完成真实点对、外参误差和人工尺量验收前，禁止把相机坐标接到抓取
Skill。当前没有自动抓取、自动移动、自动展开或自动标定动作。
