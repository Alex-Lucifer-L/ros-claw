# SO-100 Plus 离线仿真保真度说明

## 已复用的真实工程资产

- 本地 `lerobot-joycon_plus` 的 `scene_plus.xml`、`so100_plus.xml` 与 mesh；
- 项目 `SO100PlusKinematics` 的六轴 FK/IK、TCP 偏移
  `(0.10127, -0.00690, 0.00118) m`；
- 已登记的 `SO100PlusIrregularWorkspace` 网格及其端点/单元门禁；
- `SO100PlusMuJoCoTrajectoryValidator` 的关节限位、TCP 最低高度、机器人
  碰撞/接触预检。

关节映射与项目一致：`shoulder_rotation_joint`、`shoulder_pitch_joint`、
`ellbow_joint`、`wrist_pitch_joint`、`wrist_jaw_joint`、`wrist_roll_joint`，
夹爪为 `gripper_joint`。TCP 仍表示两根夹指尖端中点，单位为米、基座系
`+Z` 向上。

## 仿真假设（不可用于真机）

- `configs/simulation_camera.example.json` 的固定外参和针孔内参；
- 桌面高度、简单立方体/盒子/圆柱体、质量和摩擦；
- 夹爪接近物体后的保持/滑落判断；
- RGB/Depth 噪声、孔洞、遮挡的轻量注入。

这些值都标记为 `simulation_only: true`，代码不会读取真实 RealSense 或
`/dev/lerobot_right`。仿真成功不是、也不能表述为真机抓取成功。

## 主要 sim-to-real 差距

1. 没有实验桌和真实物体的 CAD 碰撞体、重心、摩擦与形变；
2. 没有真实电机滞后、齿隙、负载/温度/通信误差；
3. 虚拟相机不是 D435i 出厂内参，也不是已验收的真实 eye-to-hand 外参；
4. 离线 `SimulatedColorVLM` 不是千问视觉模型，不能代表真实 VLM 识别率；
5. 仿真 REST/WORK 是软件会话状态，未重新认证 follower_rest 真实过渡。

回实验室后必须重新完成：只读设备检查、真实相机内外参/时间同步、无物体
运动轨迹验收、接触/夹持小样本验收、stop 与失败恢复验收。不得导入仿真
配置、阈值或 benchmark 成功率来替代上述步骤。
