# SO-100 Plus `middle_internal` 1 cm 不规则空间快照

这是 2026-08-02 完成的纯离线扫描归档。项目后续确定以这类不规则点集和
连通路径作为工作空间方向，不再把有效点的独立 XYZ 极值误当作长方体。

## 固定输入

```text
参考姿态: middle_internal
TCP 朝向: middle_internal 的固定朝向
笛卡尔网格: 10 mm
关节路径检查步长: 1°
夹爪驱动角交集: -5° 与 60°
支撑平面: Z = 0
真实硬件访问: 0
```

完整复现命令见仓库文档：
`docs/so100_plus_middle_irregular_workspace.md`。

## 最终结果

```text
扫描点:               62,092
有效点:               10,974
IK/关节限制失败:      50,484
目标接触/碰撞:           634
路径碰撞:                  0
```

有效点独立轴向极值为：

```text
X:  0.173571 .. 0.523571 m
Y: -0.161185 .. 0.098815 m
Z:  0.039328 .. 0.379328 m
```

这三个范围只描述扇形/楔形点集的投影，不能交叉组合成可用安全框。

## 归档文件与 SHA-256

主扫描：`artifacts/so100_plus_middle_workspace_10mm_final/`

```text
42431ad66ca85aa75147366f7f89364318a005b4e6787a74c0f32c627a7c3d19  rest_workspace_report.json
c139f4e9f75343d01a368fea30dfc3d1b1d40ae13dfbc7f9f84e14f2bb34ad27  rest_workspace_grid.npz
32be1a7e67637956ce5e1a205c816149cd7b3815fe0818784757b4b7c74adfd9  rest_workspace_views.png
```

前方 X 探边：`artifacts/so100_plus_middle_workspace_20mm_frontier/`

```text
1228ac358c2988a0724c9750a5bfb297eefcd96f29d7f27b52c6d7007dd522eb  rest_workspace_report.json
e73df96dac7de289eda3076ca4bb040f24f906578e27f6d0013729b0b530a161  rest_workspace_grid.npz
7f39eb4d2af174f337bde812ce6ae99619384dfc11ca5566d6f8e86a4deef2a9  rest_workspace_views.png
```

如果文件内容发生变化，必须重新生成快照说明和校验和，不能继续把改变后
的数据称为这次扫描结果。

## NPZ 关键字段

| 字段 | 含义 |
| --- | --- |
| `x_m`, `y_m`, `z_m` | 三条笛卡尔网格轴，单位米 |
| `status` | 每个网格点的扫描状态码 |
| `valid_status_code` | 有效状态码 |
| `target_joint_radians` | 对应网格点的六关节 IK 解 |
| `valid_tcp_points_m` | 所有不规则有效 TCP 点 |
| `rest_index` | 参考点在三维网格中的索引 |
| `rest_tcp_m` | `middle_internal` TCP |
| `rest_joint_radians` | `middle_internal` 六关节角 |
| `gripper_driver_degrees` | 扫描要求同时通过的夹爪驱动角 |
| `gripper_qpos_radians` | 上述夹爪角映射到模型的 qpos |

状态码定义保存在 `workspace_scan/so100_plus.py`，读取方不应自行猜测。

## 当前认证等级

该快照是“仿真候选”，不是“真机正式认证”。当前正式运行时仍使用已有
认证长方体。将不规则空间投入真机前，还缺整个有效点集的邻接图/路径规划、
产物与硬件配置绑定、运行时同轨迹预检，以及少量代表路径的人工真机验收。
