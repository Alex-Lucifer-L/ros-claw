# V2.0 只读视觉观察

## 当前能力

V2.0 增加了一条与机械臂执行链完全分开的视觉链路：

```text
USB 摄像头或本地图像
→ OpenCV 单帧读取、等比例缩放和 JPEG 编码
→ 阿里云百炼 OpenAI-compatible 千问视觉模型
→ 严格 JSON 解析和字段校验
→ SceneObservation
→ text/json 终端输出
```

它可以描述画面、列出可见物体、回答“红色方块在画面哪一侧”这类视觉
问题。它不会生成 `Command`，不会创建 `ExecutionController`，不会调用
Adapter，也不会连接机械臂。即使命令行同时写了
`--backend so100_plus`，`--input-mode vision` 也会在创建 Arm Runtime 前
独立分流，并且不要求真机风险确认参数。

## 安装与配置

普通单帧视觉观察不需要 GUI，但棋盘采集和内参验收需要实时
预览，因此项目使用带 GUI 支持的 `opencv-python`：

```bash
python -m pip install -r requirements.txt
```

百炼配置可参考 `.env.example`，但程序不会自动读取 `.env` 文件；请把值
放入当前 shell 环境。不要把真实密钥写进仓库。

```bash
export ROSCLAW_LLM_BASE_URL="https://dashscope.aliyuncs.com/compatible-mode/v1"
export ROSCLAW_LLM_API_KEY="<你的百炼 API Key>"
export DASHSCOPE_VL_MODEL="qwen-vl-plus"
```

视觉模型优先级是：

```text
--vlm-model
→ DASHSCOPE_VL_MODEL
→ qwen-vl-plus
```

API Key 优先读取 `ROSCLAW_LLM_API_KEY`，未设置时再读取
`DASHSCOPE_API_KEY`。`ROSCLAW_LLM_BASE_URL` 未设置时使用上面的百炼兼容
地址。每次请求默认超时 30 秒，不自动无限重试。

## 查看 Ubuntu 摄像头编号

下面是操作者可自行执行的只读检查示例；开发和自动测试不会运行它们：

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

同一物理摄像头有时会暴露多个 `/dev/videoN` 节点，应选择能输出图像的
节点编号。权限错误时检查当前用户是否属于 `video` 组。

## 摄像头运行方式

交互模式：

```bash
PYTHONPATH=src python -m rosclaw_mini.main \
  --input-mode vision \
  --camera-index 0
```

可输入：

```text
observe
ask 红色方块在画面哪一侧？
exit
```

每次 `observe` 或 `ask` 只打开摄像头、读取一帧并立即释放，然后才发送
模型请求，不长期占用设备。默认不会把图像写到磁盘；只有显式传入下列
参数才保存当次捕获帧：

```bash
--vision-save-frame /tmp/rosclaw-observation.jpg
```

腕部相机应优先使用不会随枚举顺序漂移的设备路径：

```bash
export ROSCLAW_VISION_CAMERA_DEVICE="/dev/v4l/by-id/<wrist-camera>-video-index0"

PYTHONPATH=src python -m rosclaw_mini.main \
  --input-mode vision
```

也可以显式传入 `--camera-device /dev/v4l/by-id/...-video-index0`，其优先级
高于 `--camera-index`。当前机器已只读确认腕部 Sonix FHD Webcam 的默认
协商参数为 V4L2、YUYV、640 × 480、20 FPS，OpenCV 实际帧 shape 为
`(480, 640, 3)`。这些只是采集参数，不是相机内参或手眼外参。

## 腕部相机内参标定

当前棋盘格已经由真实画面确认：

```text
内角点：7 × 6
方格数：8 × 7
单格边长：24 mm = 0.024 m
棋盘图案：192 mm × 168 mm
```

先手动采集至少 15 张不同视角。程序默认打开实时预览窗口，自动检测
`7×6` 个内角点：完整识别时显示角点并提示 `READY`，按 Space 或 `C`
保存；按 `Q` 或 Esc 结束。它不会连接、读取或移动机械臂：

```bash
PYTHONPATH=src python scripts/collect_wrist_camera_calibration_images.py \
  --device /dev/v4l/by-id/<wrist-camera>-video-index0 \
  --output-dir local_calibration/wrist_camera/images \
  --count 15 \
  --width 640 \
  --height 480 \
  --acknowledge-camera-capture
```

窗口中显示 `NOT READY` 时不会保存，即使按了 Space/C 也只会在终端说明
角点未完整识别。需要无窗口采集时可额外传入 `--no-preview`，退回每张按
Enter 的方式。实时预览需要普通 `opencv-python`；项目不再使用
`opencv-python-headless`。

每一张都要满足：棋盘完整、四周留白、角点清晰；同时改变棋盘在画面中的
左/中/右、上/中/下位置、距离和倾斜方向。不要只平移或连续保存几乎相同
的画面。采集图片与结果位于 `.gitignore` 中的 `local_calibration/`，不会
作为通用配置提交。

然后离线求解内参：

```bash
PYTHONPATH=src python scripts/calibrate_wrist_camera_intrinsics.py \
  --images-dir local_calibration/wrist_camera/images \
  --output local_calibration/wrist_camera/intrinsics.json \
  --device /dev/v4l/by-id/<wrist-camera>-video-index0 \
  --vendor-id 0c58 \
  --product-id 637a \
  --serial '<本机相机序列号>' \
  --pixel-format YUYV \
  --width 640 \
  --height 480 \
  --inner-columns 7 \
  --inner-rows 6 \
  --square-size-mm 24 \
  --minimum-views 10
```

求解器拒绝少于 10 张有效图、角点不完整、分辨率混用和非有限结果。输出
包含相机矩阵、畸变系数、RMS、逐图重投影误差、接受/拒绝图片列表、设备
身份、棋盘物理尺度和 SHA-256。生成文件仍只是内参；腕部相机到夹爪/TCP
的手眼外参必须在内参通过后，用固定标定板与多组同步机械臂真实位姿另行
求解。

### 当前内参结果与独立验收

当前 Sonix FHD Webcam 的第一版内参使用 15 张 `640×480 YUYV`
图片求解，15 张全部接受，求解 RMS 为 `0.492674 px`。本地产物位于：

```text
local_calibration/wrist_camera/session_20260805_01/intrinsics.json
```

该文件绑定稳定设备路径、`0c58:637a`、序列号 `SN0001`、分辨率和
像素格式，并使用 SHA-256 防止静默修改。它位于 `.gitignore` 内，不是所有
同型号摄像头共享的通用参数。

求解后必须用未参与求解的新视角做独立验收：

```bash
PYTHONPATH=src:lerobot-joycon_plus python \
  scripts/check_wrist_camera_intrinsics.py \
  --device /dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._FHD_Webcam_SN0001-video-index0 \
  --calibration local_calibration/wrist_camera/session_20260805_01/intrinsics.json \
  --output-dir local_calibration/wrist_camera/session_20260805_01/validation \
  --count 3 \
  --pixel-format YUYV \
  --acknowledge-camera-capture
```

左侧是原图 `RAW`，右侧是 `UNDISTORTED`。将棋盘放到没有出现在前 15 张
中的新位置和新倾角；画面显示 `BOARD ERROR` 后按 Space 或 `C`
保存一组同时刻的原图/去畸变图，按 `Q` 或 Esc 结束。工具会在打开
摄像头前验证文件哈希、设备路径和像素格式，逐帧验证分辨率；不匹配时
失败关闭。该工具不导入任何机械臂模块。

## 腕部相机手眼标定

内参验收通过后，手眼标定求解以下固定变换：

```text
base_T_tcp       现有六关节 FK，将 TCP 坐标映射到底座坐标
camera_T_target  使用已验收内参和 7×6、24 mm 棋盘 PnP 求得
tcp_T_camera     OpenCV 眼在手上求解的最终结果
```

机械臂参考系明确使用项目已有的夹爪 TCP，包含现有
`SO100_PLUS_GRIPPER_TCP_OFFSET_M`；不另外假设一个“摄像头腕部点”。

### 只读手动采集原则

当前没有为手眼标定认证任意姿态的自动运动轨迹，因此程序不会自动
转动关节。采集时：

1. 标定板牢固固定在桌面，整个数据集期间不能移动；
2. 控制板保持供电和串口通信，但七个电机力矩必须全部关闭；
3. 操作者始终托住机械臂，手动改变位置和腕部倾角；
4. 至少 15 组，需要两个非平行旋转方向，不能只平移或绕同一轴旋转；
5. 每次按键后程序按“读关节 → 抓帧 → 再读关节”采样；前后任一
   关节漂移超过 `1°` 时不保存；
6. 每个姿态必须通过第三方模型关节范围、实测底座范围、夹爪映射和
   MuJoCo 静态无碰撞检查。

采集程序允许从 `follower_rest` 收纳姿态启动：启动时只读取并确认全部
力矩关闭，不会把 REST 套入普通 WORK 关节范围。上述关节、TCP 高度和碰撞
检查只在操作者按 Space/C 准备保存当前手眼样本时执行。REST 可以是进入
采集界面的起点，但不是可保存的手眼样本。

采集脚本使用 `create_so100_plus_readonly_robot()`，不会写入
`Goal_Position`、`Torque_Enable`、PID 或校准。力矩开启时直接失败关闭，
也不会帮操作者自动关闭力矩。

现场采集命令（只能在标定板已固定、力矩已关闭并托住机械臂后执行）：

```bash
PYTHONPATH=src:lerobot-joycon_plus python \
  scripts/collect_so100_plus_hand_eye_samples.py \
  --port /dev/lerobot_right \
  --calibration-dir lerobot-joycon_plus/.cache/calibration/so100_plus \
  --follower-name right \
  --camera-device /dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._FHD_Webcam_SN0001-video-index0 \
  --camera-intrinsics local_calibration/wrist_camera/session_20260805_01/intrinsics.json \
  --output-dir local_calibration/wrist_camera/session_20260805_01/hand_eye \
  --count 15 \
  --pixel-format YUYV \
  --acknowledge-readonly-hand-eye-capture
```

完整棋盘显示 `READY` 后按 Space/C 保存，Q/Esc 结束。每个成功样本都会
立即原子更新带 SHA-256 的 `hand_eye_dataset.json`，中途退出不会丢失已保存
位姿。

采集完成后离线求解，此命令不访问任何硬件：

```bash
PYTHONPATH=src:lerobot-joycon_plus python \
  scripts/calibrate_so100_plus_hand_eye.py \
  --dataset local_calibration/wrist_camera/session_20260805_01/hand_eye/hand_eye_dataset.json \
  --intrinsics local_calibration/wrist_camera/session_20260805_01/intrinsics.json \
  --output local_calibration/wrist_camera/session_20260805_01/hand_eye/tcp_T_camera.json
```

求解器至少要求 10 组数据，会拒绝内参哈希/摄像头不匹配、旋转轴退化、
无限矩阵和被修改的数据集。输出包含 `tcp_T_camera`、固定标定板
`base_T_target` 的逐样本一致性误差和哈希。求解成功仍不会自动把外参接入
机械臂运动；必须再用留出视角做坐标复算验收。

一次观察后直接退出：

```bash
PYTHONPATH=src python -m rosclaw_mini.main \
  --input-mode vision \
  --camera-index 0 \
  --vision-question "夹爪附近有什么？"
```

## 本地图像运行方式

本地图像模式不会打开摄像头：

```bash
PYTHONPATH=src python -m rosclaw_mini.main \
  --input-mode vision \
  --vision-image /path/to/test.jpg \
  --vision-question "桌面上有哪些物体？"
```

可以用 `--vision-max-width 1280` 调整上传前的最大宽度，图像始终保持原始
宽高比。`--vision-save-frame` 只适用于摄像头捕获，不能和
`--vision-image` 同时使用。

## 输出

默认 `--vision-output-format text` 便于人阅读；使用
`--vision-output-format json` 可得到完整 `SceneObservation` JSON。结构为：

```text
SceneObservation
├── observation_id
├── timestamp
├── scene_description
├── objects[]
│   ├── name
│   ├── category（可选）
│   ├── color（可选）
│   ├── location_in_image
│   ├── confidence（0..1 或 null）
│   ├── attributes
│   └── bounding_box（[x_min,y_min,x_max,y_max] 或 null）
├── warnings[]
├── source
├── model
└── raw_response（默认不保留）
```

`location_in_image` 只能表达 left/center/right 等画面相对区域。
`bounding_box` 是 VLM 的归一化语义估计，不是目标检测器的精密结果，不能
用于抓取。解析器会拒绝非法枚举、置信度、边界框以及机械臂/基座
`x/y/z` 三维坐标字段；它不会静默修补模型错误。

## 安全边界与 V2.1

当前腕部 USB 相机已完成独立单帧读取、15 视角内参求解和带哈希的
第一版本地内参文件；仍需操作者完成三张新视角的原图/去畸变画面验收。
真实百炼视觉 API 组合验收尚未完成。手眼只读采集和离线求解
软件已完成，但尚未采集真实多姿态数据或验收真实 `tcp_T_camera`。

V2.0 只实现“看”。以下功能明确未实现：GroundingDINO、AprilTag、深度
相机、手眼标定、像素到机械臂坐标转换、`SceneState`、
`TaskPlan`、自动抓取、视觉伺服、连续视频推理和 VLM 自动控制机械臂。

要进入 V2.1，至少还需要经过目标检测精度评估、深度测量、相机标定、
手眼标定和坐标转换的独立设计与现场验收。不能把本阶段的物体描述或
bounding box 直接连接到现有运动 Skill。
