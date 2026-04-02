# Crowd Statistics Node（增强版）功能与核心逻辑

## 1. 功能概述

该 ROS2 节点基于 **YOLOv8 + 双激光雷达**，在移动机器人上实现多摄像头行人检测与人群统计分析。主要特性：

- 支持 **4 个 RGB 相机**（前、左、右、后），使用 YOLOv8 ONNX 模型检测行人。
- 融合 **前/后激光雷达** 点云数据，通过角度匹配获得行人的精确距离和位置。
- 将行人位置转换到 **机器人基坐标系（base_link）**，并可选择转换到 **地图坐标系（map）** 发布。
- 每 **500ms** 汇总所有相机检测结果，计算：
  - 总人数、各相机独立计数
  - 全局密度（人数 / 圆形检测区域面积）
  - 平均局部密度（高斯核密度估计，半径可调）
  - 人群压力（密度 × 距离加权）
- 提供 **可视化 MarkerArray**（检测范围圆柱、行人球体、距离标签、相机计数标签）和 **PoseArray**。
- 支持 **调试模式**（打印角度信息）和 **可视化开关**。

## 2. 核心逻辑

### 2.1 整体数据流

图像消息 → YOLO检测 → 获取时间同步的激光扫描 →
激光点转换到相机坐标系 → 计算检测框角度范围 →
在角度容差内搜索最近激光点 → 计算人员位置（base_link）→
（可选）转换到 map 坐标系 → 存储各相机最新人员列表 →
定时器汇总 → 计算统计指标 → 发布结果与可视化


### 2.2 关键模块详解

#### 2.2.1 YOLO 检测器 (`YoloDetector`)
- 使用 OpenCV DNN 加载 YOLOv8 ONNX 模型。
- 支持 **CUDA** 后端（GPU）或 **CPU** 回退。
- 只保留类别 `person`（`class_id=0`）的检测框。
- 内部 NMS 阈值 `0.65`，内部置信度阈值 `0.15`，最终外部阈值由参数 `confidence_threshold` 控制（默认 `0.4`）。

#### 2.2.2 多相机与激光雷达时间同步
- 每个相机独立订阅压缩图像话题，在回调中执行检测。
- 激光雷达消息分别缓存为队列（前后雷达独立，队列大小 50）。
- 在图像回调中，根据图像时间戳找到最接近的激光扫描，时间差 ≤ 0.3 秒，否则丢弃该帧检测。

#### 2.2.3 激光点到相机坐标系的转换
- 使用 TF2 查询 `camera_frame` 到 `laser_frame` 的二维平面变换。
- 计算每个激光点在相机坐标系下的方位角 `angle` 和距离 `dist`。
- 过滤掉距离大于 `distance_threshold`（默认 20m）的点。

#### 2.2.4 行人测距（角度匹配）
- **角度范围计算**：根据检测框中心横坐标和宽度，结合相机水平视场角（`h_fov_rad`）计算左/右边界角度。  
  *关键修复*：在 `computeBboxAngles` 中增加了负号，使图像左侧对应相机左侧（正角度），修正了之前的映射错误。
- **激光点匹配**：在角度范围 ±`angle_tolerance` 内搜索激光点，选取距离最近的作为该行人的测距点。
- **位置转换**：将匹配到的激光点原始极坐标转换到 `base_link` 坐标系，得到行人三维位置。

#### 2.2.5 坐标转换与发布
- **base_link 坐标**：发布到 `detected_pedestrians` 话题（`PointStamped`）。
- **map 坐标**：若 `publish_map_frame=true`，同时转换到地图坐标系并发布到 `detected_pedestrians_map`。
- 所有人员位置汇总后发布 `person_poses`（`PoseArray`）。

#### 2.2.6 统计指标计算
- **全局密度**：`总人数 / (π × distance_threshold²)`
- **局部密度**（对每个人）：
- local_density = Σ exp(-d_ij² / (2σ²)) / (π × density_radius²)
其中 σ = density_radius / 3，d_ij 为两两间距


#### 2.2.7 可视化与调试
- 发布 `MarkerArray` 包含：
- 检测范围圆柱体（半透明）
- 各相机计数标签（在机器人上方垂直排列）
- 行人球体（不同相机不同颜色）
- 行人距离标签（包含相机来源）
- 若 `debug_mode=true`，在图像回调中打印每个检测框的中心百分比和角度范围（弧度）。
- 可独立关闭可视化（`publish_visualization=false`）。

## 3. 参数说明

所有参数可通过 ROS2 参数服务器配置（launch 文件或命令行）。

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `model_path` | string | `/capella/.../yolov8l.onnx` | YOLO 模型文件路径 |
| `distance_threshold` | double | 20.0 | 人员检测范围半径（米） |
| `confidence_threshold` | double | 0.4 | YOLO 检测置信度阈值 |
| `base_frame` | string | `base_link` | 机器人基坐标系名 |
| `map_frame` | string | `map` | 地图坐标系名（用于转换） |
| `publish_map_frame` | bool | true | 是否发布地图坐标系下的人员位置 |
| `use_gpu` | bool | true | 是否使用 CUDA 加速 |
| `frame_skip` | int | 1 | 跳帧数（每 `frame_skip+1` 帧处理一次） |
| `h_fov_rad` | double | 1.0472 (60°) | 相机水平视场角（弧度） |
| `debug_mode` | bool | false | 是否打印调试信息（角度等） |
| `default_person_width_rad` | double | 0.20 | 默认行人角宽度（弧度，用于小目标） |
| `density_radius` | double | 2.0 | 局部密度计算半径（米） |
| `publish_visualization` | bool | true | 是否发布可视化 Marker |
| `angle_tolerance` | double | 0.10 | 激光点匹配角度容差（弧度） |
| `camera_topic_front` | string | `/rgb_camera_front/compressed` | 前相机压缩图像话题 |
| `camera_topic_left` | string | `/rgb_camera_left/compressed` | 左相机压缩图像话题 |
| `camera_topic_right` | string | `/rgb_camera_right/compressed` | 右相机压缩图像话题 |
| `camera_topic_back` | string | `/rgb_camera_back/compressed` | 后相机压缩图像话题 |
| `camera_frame_front` | string | `front_camera_color_frame` | 前相机的 TF 坐标系名 |
| `camera_frame_left` | string | `left_camera_color_frame` | 左相机的 TF 坐标系名 |
| `camera_frame_right` | string | `right_camera_color_frame` | 右相机的 TF 坐标系名 |
| `camera_frame_back` | string | `back_camera_color_frame` | 后相机的 TF 坐标系名 |
| `front_scan_topic` | string | `/front_scan` | 前激光雷达话题 |
| `back_scan_topic` | string | `/back_scan` | 后激光雷达话题 |

## 4. 输出话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `detected_pedestrians` | `PointStamped` | 每个检测到的人员在 `base_link` 下的位置（单独发布） |
| `detected_pedestrians_map` | `PointStamped` | 若启用，发布人员在 `map` 坐标系下的位置 |
| `person_poses` | `PoseArray` | 所有人员的位姿数组（`base_link` 坐标系） |
| `crowd_statistics` | `Float64MultiArray` | 统计数据：`[总人数, 全局密度, 平均局部密度, 人群压力, 前相机人数, 左相机人数, 右相机人数, 后相机人数]` |
| `crowd_visualization` | `MarkerArray` | 可视化 Marker（范围圆柱、相机计数标签、行人球体、距离标签） |

## 5. 注意事项

1. **TF 依赖**：需要正确发布 `base_link`、各 `camera_frame`、`laser_frame` 以及 `map`（若启用）之间的坐标变换。
2. **时间同步**：激光雷达与图像的时间戳差异应 ≤ 0.3 秒，否则会丢弃该帧检测。
3. **性能**：YOLOv8l 模型在 GPU 上推理约 20-30ms，配合跳帧（`frame_skip=1`）可降低计算负载。定时器周期 500ms，统计频率较低。
4. **人员去重**：当前实现 **未对不同相机检测到的同一行人进行去重**，适用于相机视野不重叠的场景（例如前后左右四方向独立覆盖）。若相机视野有重叠，会导致重复计数。
5. **角度映射修复**：代码中 `computeBboxAngles` 添加了负号，修正了图像坐标到相机角度的映射，确保图像左侧对应正角度（相机左侧）。
6. **可视化坐标**：不再取反坐标，直接使用 `position_base` 发布，便于在 RViz 中直接显示。
7. **调试模式**：开启后会在控制台输出每个检测框的角度信息，有助于验证 FOV 和角度容差设置是否合理。
   
