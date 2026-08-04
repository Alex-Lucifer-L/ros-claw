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

本任务不需要 GUI 显示，因此使用无 GUI 的 OpenCV：

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

V2.0 只实现“看”。以下功能明确未实现：GroundingDINO、AprilTag、深度
相机、相机内参、手眼标定、像素到机械臂坐标转换、`SceneState`、
`TaskPlan`、自动抓取、视觉伺服、连续视频推理和 VLM 自动控制机械臂。

要进入 V2.1，至少还需要经过目标检测精度评估、深度测量、相机标定、
手眼标定和坐标转换的独立设计与现场验收。不能把本阶段的物体描述或
bounding box 直接连接到现有运动 Skill。
