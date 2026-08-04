# SO-100 Plus middle_internal 不规则仿真可达空间

本文记录以当前 `middle_internal` 为参考姿态进行的纯离线 MuJoCo 和 IK
扫描。扫描没有打开串口、创建真实 Robot、启用力矩或发送电机目标。

扫描实现、归档快照、跨机械臂迁移方法和未来 LLM 工具边界现已集中在：

- `src/rosclaw_mini/workspace_scan/README.md`
- `src/rosclaw_mini/workspace_scan/SO100_PLUS_MIDDLE_INTERNAL_10MM.md`

> 这是固定 TCP 朝向下的仿真候选点集，不是真机认证，也不能把点集的
> XYZ 外包围盒直接写成 `WorkspaceLimits`。

## 扫描语义

每个 1 cm 笛卡尔网格点必须同时满足：

- 从 `middle_internal` 的六关节姿态求得保持相同 TCP 朝向的 IK；
- 底座使用实测驱动范围，其余关节使用 MuJoCo 模型范围；
- 目标姿态不自碰撞、不接触 `Z=0` 支撑平面；
- 从 `middle_internal` 到目标的最大 1° 关节插值路径全程无碰撞；
- 夹爪关闭 `-5°` 和打开 `60°` 两种驱动姿态都通过。

因此有效点表示“从当前 WORK 中心可以沿已检查路径到达”的固定朝向
候选空间。它不是允许 TCP 任意旋转时的所有位置并集。

## 最终 1 cm 精扫结果

扫描网格共 `62,092` 点：

| 结果 | 数量 |
| --- | ---: |
| 路径可达且两种夹爪姿态无碰撞 | 10,974 |
| IK 或关节范围失败 | 50,484 |
| 终点接触/碰撞 | 634 |
| 路径碰撞 | 0 |

10,974 个有效点构成明显弯曲的扇形/楔形空间。有效点的独立轴向范围为：

```text
X:  0.173571 .. 0.523571 m
Y: -0.161185 .. 0.098815 m
Z:  0.039328 .. 0.379328 m
```

这些只是点集的三个独立极值，不能组合成一个长方体。不同 X 切片的
横截面差异很大：

| X | 有效点数 | Y 范围 | Z 范围 |
| ---: | ---: | ---: | ---: |
| 0.174 m | 3 | -0.011 .. 0.009 m | 0.299 m |
| 0.274 m | 82 | -0.031 .. 0.019 m | 0.239 .. 0.379 m |
| 0.374 m | 428 | -0.091 .. 0.049 m | 0.069 .. 0.359 m |
| 0.474 m | 608 | -0.161 .. 0.089 m | 0.039 .. 0.279 m |
| 0.524 m | 119 | -0.061 .. 0.059 m | 0.079 .. 0.179 m |

按 1 cm 体素近似，有效点集体积约 `0.010974 m³`。该数值受网格分辨率
影响，只用于比较，不是真实连续体积证明。

当前正式 `12 × 6 × 12 cm` 长方体包含其中 1,183 个有效网格点；另有
9,791 个有效点位于当前长方体之外。这证明当前正式框只是仿真可达空间
中的一个小型保守子区，不代表机械臂完整工作空间。

## 前方 X 探边

最终 1 cm 扫描在 `X=0.523571 m` 仍有有效点。另用允许超出旧随机样本
AABB 的 2 cm 网格把 X 扫到 0.65 m；从约 `X=0.533571 m` 起不再出现
有效点。因此当前分辨率下，固定朝向前端边界位于约：

```text
0.524 m <= X_max < 0.534 m
```

## 参考长方体只作统计

算法同时找到一个包含 `middle_internal`、所有 1 cm 网格点均通过的最大
轴对齐长方体：

```text
X:  0.363571 .. 0.483571 m  (12 cm)
Y: -0.091185 .. 0.048815 m  (14 cm)
Z:  0.089328 .. 0.249328 m  (16 cm)
```

框内 18,548 条相邻网格有向边全部通过重新 IK 和碰撞路径检查。这个框
明显不对称，且仍然只属于仿真结果；本次不把它替换为正式工作空间。

## 输出文件

最终精扫结果：

- `artifacts/so100_plus_middle_workspace_10mm_final/rest_workspace_report.json`
- `artifacts/so100_plus_middle_workspace_10mm_final/rest_workspace_grid.npz`
- `artifacts/so100_plus_middle_workspace_10mm_final/rest_workspace_views.png`

NPZ 中的 `valid_tcp_points_m` 是 10,974 个不规则有效 TCP 点；`status` 和
`target_joint_radians` 保留整个网格的结果与对应 IK 解。

前端扩展探边结果位于：

- `artifacts/so100_plus_middle_workspace_20mm_frontier/`

## 复现命令

```bash
PYTHONPATH=src:lerobot-joycon_plus \
MPLCONFIGDIR=/tmp/matplotlib-rosclaw \
python scripts/simulate_so100_plus_rest_workspace.py \
  --reference-pose middle_internal \
  --gripper-driver-degrees -5 60 \
  --grid-step-mm 10 \
  --path-step-degrees 1 \
  --bounds-m \
    0.1535714232672181 0.529 \
    -0.3115 0.10881450571983638 \
    0.01932848288990053 0.39932848288990055 \
  --output-dir artifacts/so100_plus_middle_workspace_10mm_final
```

## 仍需真机确认

仿真没有表达回差、重力下垂、线缆、负载、装配误差和外部障碍物。
如果以后要把不规则点集用于正式运行，应先设计基于网格/体素和连通路径
的安全检查，随后只选择少量代表点进行人工真机验收。不能把上述 AABB
极值直接塞进现有长方体限制。

项目后续已确定采用不规则离散安全空间，而不是扩大后的 AABB。该决定目前
只确定了技术方向；在全点集连通图、运行时规划、配置绑定和人工真机认证
完成前，现有正式工作空间门禁保持不变。
