cmake_minimum_required(VERSION 3.8)
project(crowd_statistics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包 - 添加 visualization_msgs
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)  # 添加这一行

# 包含目录
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(crowd_statistics_node src/crowd_statistics.cpp)

# 链接依赖 - 添加 visualization_msgs
ament_target_dependencies(crowd_statistics_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  visualization_msgs  # 添加这一行
)

target_link_libraries(crowd_statistics_node
  ${OpenCV_LIBS}
)

# 安装目标
install(TARGETS
  crowd_statistics_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件（如果有）
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
