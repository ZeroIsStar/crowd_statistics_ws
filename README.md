# Crowd Statistics Node

## 概述

`crowd_statistics_node` 是一个 ROS2 C++ 节点，利用四个单目摄像头（前、左、右、后）和前后激光雷达，实现对机器人周围行人的检测、定位与人群统计。该节点基于 YOLOv8 ONNX 模型进行人体检测，通过激光雷达提供精确距离，融合多传感器数据，输出总人数、人群密度和压力指标，并提供丰富的可视化信息。

## 功能

- **行人检测**：使用 YOLOv8l ONNX 模型对四个摄像头的图像进行推理，获取人体边界框。
- **激光测距**：订阅前/后激光雷达扫描，通过 TF 变换将激光点投影到相机坐标系，匹配检测框角度范围，获得行人的精确三维位置。
- **多相机融合**：将来自不同相机的检测结果转换到机器人坐标系（`base_link`），基于欧氏距离去重合并，提高定位鲁棒性。
- **人群统计**：
  - **全局密度**：总人数 / (π × 检测半径²)
  - **局部密度**：每个行人周围半径 `density_radius` 内其他行人的高斯加权密度
  - **人群压力**：综合考虑局部密度和行人到机器人距离的加权平均值，反映拥挤程度和靠近机器人的危险性
- **坐标输出**：
  - 在 `base_link` 坐标系下实时发布行人位置
  - 可选地在 `map` 坐标系下发布（需 TF 支持）
- **可视化**：发布 MarkerArray，显示检测范围圆柱体、行人球体（颜色区分相机）、距离标签。
- **参数灵活配置**：支持调整检测半径、置信度阈值、去重距离、视场角等，适应不同场景。

## 核心算法

### 1. YOLO 检测器（`YoloDetector`）
- 输入：`cv::Mat` 图像帧
- 处理：
  - 缩放至 640×640，归一化后送入 OpenCV DNN 网络。
  - 解析输出张量，支持 `[1,84,8400]` 或 `[1,8400,84]` 两种常见格式。
  - 对每个候选框提取类别为 person（class_id=0）且置信度 > 0.15 的框。
  - 执行 NMS（IoU 阈值 0.65）去除重叠框，并返回检测结果（含边界框、中心点坐标、置信度）。
- 特性：检测框最小宽高限制为 30 像素，避免无效框。

### 2. 激光点投影与角度匹配
- **`transformLaserToCamera`**：通过 TF 查找从激光雷达坐标系到相机坐标系的变换，将每个激光点转换到相机坐标系，计算水平角 `atan2(yc, xc)` 和距离，并保留原始极坐标用于后续位置重建。
- **`computeBboxAngles`**：根据检测框中心像素坐标和水平视场角（`h_fov_rad`）计算中心角度，并利用边界框宽度（或默认行人宽度）确定左右边界角度。关键修正：添加负号使图像左侧对应正角度（相机坐标系左为正）。
- **`findNearestInAngleRange`**：在激光点中搜索角度落在 `[left_angle, right_angle] ± angle_tolerance` 范围内的最近点，提供匹配的行人距离。

### 3. 多相机融合与去重
- **`mergeAndDeduplicate`**：
  - 聚类：遍历所有候选行人，若两行人来自不同相机且欧氏距离 < `merge_distance`，归为同一簇。
  - 代表点选择：簇内选择综合得分（置信度 / (1+距离)）最高的行人作为代表，并可选择平均位置（当簇内点数 >1 时）。
  - 返回唯一行人列表。

### 4. 密度与压力计算
- **全局密度**：`total_people / (π * distance_threshold²)`
- **局部密度**：对每个行人，计算其周围 `density_radius` 米内其他行人的高斯加权贡献：
  \[
  \text{density} = \sum_{j \neq i} \exp\left(-\frac{d_{ij}^2}{2\sigma^2}\right) / (\pi r^2)
  \]
  其中 \( \sigma = r/3 \)，\( r = \text{density\_radius} \)。
- **人群压力**：对所有行人，将局部密度乘以因子 `(1 + 0.1 * dist_from_center)` 后取平均，即：
  \[
  \text{pressure} = \frac{1}{N} \sum_i \text{local\_density}_i \times (1 + 0.1 \cdot \text{dist}_i)
  \]

## 参数说明

| 参数 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `model_path` | string | `/capella/.../yolov8l.onnx` | YOLOv8 ONNX 模型文件路径 |
| `distance_threshold` | double | 20.0 | 检测半径（米） |
| `confidence_threshold` | double | 0.4 | 行人检测置信度阈值 |
| `merge_distance` | double | 0.8 | 多相机去重距离（米） |
| `base_frame` | string | `base_link` | 机器人本体坐标系 |
| `map_frame` | string | `map` | 世界坐标系（用于可选发布） |
| `publish_map_frame` | bool | true | 是否发布 `map` 坐标系下的位置 |
| `use_gpu` | bool | true | 是否启用 CUDA 加速（OpenCV DNN） |
| `frame_skip` | int | 1 | 跳帧数（每 `frame_skip+1` 帧处理一次） |
| `h_fov_rad` | double | 1.0472 | 相机水平视场角（弧度，默认 60°） |
| `debug_mode` | bool | false | 开启详细调试输出 |
| `default_person_width_rad` | double | 0.20 | 当检测框过小时使用的默认行人角度宽度（弧度） |
| `density_radius` | double | 2.0 | 局部密度计算半径（米） |
| `publish_visualization` | bool | true | 是否发布可视化 MarkerArray |
| `angle_tolerance` | double | 0.10 | 角度匹配容差（弧度） |

相机和激光雷达话题名也通过参数配置（默认与第一个代码一致）。

## 发布话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `detected_pedestrians` | `geometry_msgs::msg::PointStamped` | 在 `base_link` 坐标系下的实时行人位置 |
| `detected_pedestrians_map` | `geometry_msgs::msg::PointStamped` | 在 `map` 坐标系下的实时行人位置（若 `publish_map_frame` 为 true） |
| `person_poses` | `geometry_msgs::msg::PoseArray` | 所有行人的位姿（位置，姿态为默认朝向） |
| `crowd_statistics` | `std_msgs::msg::Float64MultiArray` | 统计信息：`[总人数, 全局密度, 平均局部密度, 压力]` |
| `crowd_visualization` | `visualization_msgs::msg::MarkerArray` | 可视化标记（范围圆柱、行人球体、文字标签） |

## 可视化说明

- **范围圆柱体**：半透明青色圆柱，半径 = `distance_threshold`，高 1m，原点在机器人中心（z = -0.5 使底部接地），显示检测范围。
- **行人球体**：半径 0.4m，颜色按相机 ID 区分（前:红，左:绿，右:蓝，后:黄），半透明。
- **文字标签**：显示行人索引和距离（如 `P0: 3.5m`），位于行人正上方。

每次定时器回调时，首先发布一个 `DELETEALL` 标记清除旧可视化，再发布新的标记。

## 依赖

- ROS2 Humble（或更高版本）
- OpenCV 4.x（带 dnn 模块）
- cv_bridge
- tf2_ros
- sensor_msgs, geometry_msgs, visualization_msgs, std_msgs
- YOLOv8 ONNX 模型（需与代码中格式兼容）

## 编译与运行

1. 将代码保存为 `crowd_statistics_node.cpp`，并在 `CMakeLists.txt` 中添加可执行文件，链接所需库。
2. 编译：
   ```bash
   colcon build --packages-select your_package_name
   source install/setup.bash
