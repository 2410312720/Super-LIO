# Super-LIO 动态点云滤除

## 目标与数据流

该功能用于降低行人、车辆等动态点对位姿更新和 OctVoxMap 的影响。功能默认关闭；开启后的处理链为：

```text
去畸变 → 降采样 → OctVoxMap KNN/平面拟合 → 几何残差分类
       → 动态软权重 + Cauchy 核 → ESKF 更新 → 最终标签
       → 静态/未知/动态写图分流 → 世界坐标调试点云
```

实现只复用现有降采样点、KNN 和平面系数，不改变 OctVoxMap 内部格式。当前保存的是降采样静态地图，不是带完整标签传播的高密静态地图。

## 标签与几何证据

`MotionLabel` 有四种状态：

- `UNKNOWN`：没有有效局部平面，或残差位于静态和动态门限之间。
- `STATIC_CONFIRMED`：几何残差足够小。
- `DYNAMIC_CANDIDATE`：几何残差达到动态候选门限。
- `DYNAMIC_CONFIRMED`：为后续时序动态确认预留；第一版几何分类不会主动产生该标签，但权重、写图和调试路径均支持它。

对距离为 `range` 的点，基础残差门限为：

```text
threshold = residual_base + residual_range_scale × range
```

分类规则为：

```text
无有效平面                                      → UNKNOWN
abs(residual) <= threshold                      → STATIC_CONFIRMED
abs(residual) >= dynamic_residual_multiplier
                × threshold                     → DYNAMIC_CANDIDATE
其余                                             → UNKNOWN
```

KNN 有效、平面有效、原始硬残差内点和最终残差分别存储。平面有效时，即使没有通过 Super-LIO 原有硬门限，也会保留残差和调试标签。

## 位姿更新、鲁棒核与 fallback

动态位姿过滤开启时，进入信息矩阵的权重为：

```text
cauchy_weight = 1 / (1 + (residual / robust_scale)²)
omega = observation_inv_variance × label_weight × cauchy_weight
```

默认标签权重为静态 `1.0`、未知 `0.25`、动态候选 `0.05`、动态确认 `0.0`。非有限残差、非法尺度或非有限权重不会进入信息矩阵。原有 Jacobian、`HTVH` 和 `HTVr` 的符号保持不变。

并行计算同时形成一份 fallback 信息矩阵。fallback 仍要求通过原始硬残差门限并使用 Cauchy 核，但不使用运动标签降权。当动态加权实际进入累加的点数少于 `min_effective_points` 时，本次 ESKF 迭代使用 fallback，并将该 LiDAR 帧标记为 tracking degraded。degraded 帧不会更新主地图或时序体素，避免用退化位姿污染地图。

当 `lio.dynamic.enable=false` 或 `filter_pose_update=false` 时，Observe 继续使用原始固定 1000 权重，不触发动态 fallback。

## UNKNOWN 延迟写图

动态地图过滤开启时：

- `STATIC_CONFIRMED` 立即写入 OctVoxMap；
- `UNKNOWN` 先进入 `TemporalVoxelMap`；
- `DYNAMIC_CANDIDATE` 和 `DYNAMIC_CONFIRMED` 不写图；
- tracking degraded 帧不更新任何地图。

`TemporalVoxelMap` 使用世界坐标体素。当前帧会先在体素内聚合，因此同一帧同一体素无论有多少点都只增加一次帧观测。体素必须连续出现至少 `static_confirm_frames` 帧，且位置方差不大于 `max_static_variance`，才会提升一个均值代表点。每个体素只提升一次；长时间未见且尚未提升的体素会在 `max_unseen_frames` 后删除。

## 初始化

动态地图过滤开启时，初始化不再逐帧直接写入全部点，而是：

1. 收集 `init_collect_frames` 帧；
2. 每帧先转到初始化世界坐标，再按 `temporal_voxel_size` 聚合；
3. 每个体素按不同帧统计命中次数和代表点位置方差；
4. 命中至少 `init_static_min_hits` 且方差合格的体素作为静态种子；
5. 一次性写入 OctVoxMap。

如果静态种子少于 `init_min_static_points`，节点会打印 warning，并使用所有初始化帧经体素聚合后的代表点作为安全回退，避免系统因空初始地图无法运行。动态地图过滤关闭时，保留原始四帧初始化行为。

## 地图保存

`filter_saved_map=true` 时，仅保存最终标签为 `STATIC_CONFIRMED` 的降采样点和本帧新提升的时序静态代表点。`UNKNOWN` 和两种动态标签不会保存，已是世界坐标的新提升点不会重复应用位姿。关闭动态功能或设置 `filter_saved_map=false` 时，保留原有高密/降采样保存选择。

## 重定位模式

重定位复用基类 Observe、分类、fallback、地图分流和调试发布：

- `lio.relocation.update_map=false`：继续进行动态识别和位姿降权，可发布调试点云，但不更新 OctVoxMap 或 `TemporalVoxelMap`；
- `lio.relocation.update_map=true`：保留原有 100 帧更新延迟，延迟结束后使用与普通模式相同的静态/未知/动态分流；
- 加载已有 PCD 地图、NDT/ICP 初始化流程不变；
- tracking degraded 帧始终禁止写图。

## ROS 参数

所有参数位于 `lio.dynamic`，示例见 `config/livox_360.yaml` 和 `config/relocation.yaml`。

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `enable` | `false` | 动态功能总开关 |
| `filter_pose_update` | `true` | 对 ESKF 观测应用动态软权重 |
| `filter_map_update` | `true` | 对 OctVoxMap 写入应用标签分流 |
| `filter_saved_map` | `true` | 保存时只保留静态点 |
| `publish_debug_clouds` | `true` | 发布分类调试话题 |
| `temporal_voxel_size` | `0.25` | 时序与初始化体素尺寸，米 |
| `static_confirm_frames` | `3` | UNKNOWN 连续静态确认帧数 |
| `max_unseen_frames` | `20` | 未提升体素清理间隔 |
| `max_static_variance` | `0.01` | 静态体素最大位置方差 |
| `residual_base` | `0.10` | 距离无关残差门限，米 |
| `residual_range_scale` | `0.003` | 随距离增长的残差门限系数 |
| `dynamic_residual_multiplier` | `2.5` | 动态候选门限倍数 |
| `robust_scale` | `0.15` | Cauchy 尺度，米 |
| `observation_inv_variance` | `1000.0` | 观测基础逆方差 |
| `static_weight` | `1.0` | 静态标签权重 |
| `unknown_weight` | `0.25` | 未知标签权重 |
| `candidate_weight` | `0.05` | 动态候选权重 |
| `dynamic_weight` | `0.0` | 动态确认权重 |
| `min_effective_points` | `40` | fallback 最小动态加权点数 |
| `init_collect_frames` | `6` | 初始化收集帧数 |
| `init_static_min_hits` | `3` | 初始化静态体素最少帧命中数 |
| `init_min_static_points` | `100` | 初始化静态种子最少数量 |

非法体素尺寸、尺度、帧数或权重会在参数加载后被修正为安全值，并打印 warning。

## 调试话题与 RViz

动态功能和调试发布同时开启时，节点发布：

- `/lio/cloud_static` (`sensor_msgs/msg/PointCloud2`)
- `/lio/cloud_dynamic` (`sensor_msgs/msg/PointCloud2`)，包含两种动态标签
- `/lio/cloud_unknown` (`sensor_msgs/msg/PointCloud2`)
- `/lio/dynamic_ratio` (`std_msgs/msg/Float32`)

三类点云均为 `world` 坐标系，时间戳取当前 LiDAR 帧。可在 RViz 中添加三个 PointCloud2 显示并分别设置绿色、红色、黄色；保留 `/lio/cloud_world` 用于对照。开启方式：

```yaml
lio.dynamic.enable: true
lio.dynamic.publish_debug_clouds: true
```

## 调参建议

- 动态点漏检时，可适度减小 `dynamic_residual_multiplier` 或 `residual_base`；静态边缘误检时反向调整。
- 位姿约束过少或频繁 fallback 时，可提高 `unknown_weight`、`candidate_weight`，或降低 `min_effective_points`。
- 新区域迟迟不写图时，可减少 `static_confirm_frames` 或适度增大 `max_static_variance`。
- 时序体素过于敏感时增大 `temporal_voxel_size`；薄结构被合并时减小它。
- 应结合 `/lio/dynamic_ratio`、三类点云和实际场景逐项调整，不应仅以地图视觉稀疏度判断定位精度。

## 已知限制

当前版本尚未实现：

- 完整 free-space ray casting；
- 目标级动态聚类和跟踪；
- 语义识别；
- M-detector 逐点遮挡检测；
- 完整高密点云标签传播；
- 历史污染地图删除。

此外，长时间静止的人或车辆可能被视为静态；大面积动态遮挡可能使有效约束不足并触发 fallback。已提升体素的去重标记会保留到节点结束，以保证同一体素不会重复提升，因此极长距离运行时仍需关注其内存占用。
