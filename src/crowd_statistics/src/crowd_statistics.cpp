#include "crowd_statistics/crowd_statistics.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>

using namespace std::chrono_literals;

namespace crowd_statistics
{

// -------------------- YoloDetector 实现 --------------------
YoloDetector::YoloDetector(const std::string &model_path, const rclcpp::Logger &logger, bool use_gpu)
    : logger_(logger), enabled_(false), use_gpu_(use_gpu)
{
  try {
    net_ = cv::dnn::readNet(model_path);
    if (net_.empty()) {
      RCLCPP_ERROR(logger_, "Failed to load YOLO model: %s", model_path.c_str());
      return;
    }

    if (use_gpu_) {
      try {
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        net_.enableWinograd(false); // 修复 CUDA NaN 问题
        RCLCPP_INFO(logger_, "YOLO model loaded with CUDA backend");
      } catch (const std::exception &e) {
        RCLCPP_WARN(logger_, "CUDA backend failed, falling back to CPU: %s", e.what());
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      }
    } else {
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      RCLCPP_INFO(logger_, "YOLO model loaded with CPU backend");
    }

    enabled_ = true;
    RCLCPP_INFO(logger_, "YOLO model loaded: %s", model_path.c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Exception while loading YOLO model: %s", e.what());
  }
}

std::vector<Detection> YoloDetector::detect(const cv::Mat &frame)
{
  std::vector<Detection> detections;
  if (!enabled_ || frame.empty()) return detections;

  constexpr int input_size = 640;
  cv::Mat blob;
  cv::dnn::blobFromImage(frame, blob, 1.0/255.0,
                         cv::Size(input_size, input_size),
                         cv::Scalar(), true, false);
  net_.setInput(blob);

  std::vector<cv::Mat> outputs;
  net_.forward(outputs, net_.getUnconnectedOutLayersNames());
  if (outputs.empty()) return detections;

  const cv::Mat &out = outputs[0];
  if (out.dims != 3) return detections;

  // YOLOv8 输出 [1, 84, 8400] → 转置为 [8400, 84]
  cv::Mat det;
  if (out.size[1] < out.size[2]) {
    cv::Mat raw(out.size[1], out.size[2], CV_32F,
                const_cast<float *>(out.ptr<float>()));
    cv::transpose(raw, det);
  } else {
    det = cv::Mat(out.size[1], out.size[2], CV_32F,
                  const_cast<float *>(out.ptr<float>())).clone();
  }

  const int class_count = det.cols - 4;
  if (class_count <= 0) return detections;

  const float x_scale = static_cast<float>(frame.cols) / static_cast<float>(input_size);
  const float y_scale = static_cast<float>(frame.rows) / static_cast<float>(input_size);

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> class_ids;

  for (int i = 0; i < det.rows; ++i) {
    const float *row = det.ptr<float>(i);
    if (!row) continue;

    int best_class = -1;
    float best_score = 0.0F;
    for (int c = 0; c < class_count; ++c) {
      float score = row[4 + c];
      if (score > best_score) {
        best_score = score;
        best_class = c;
      }
    }

    if (best_class != 0 || best_score < 0.25F) continue;

    const float cx = row[0] * x_scale;
    const float cy = row[1] * y_scale;
    const float w  = row[2] * x_scale;
    const float h  = row[3] * y_scale;

    int x1 = static_cast<int>(cx - 0.5F * w);
    int y1 = static_cast<int>(cy - 0.5F * h);
    int bw = static_cast<int>(w);
    int bh = static_cast<int>(h);

    boxes.emplace_back(x1, y1, bw, bh);
    confidences.push_back(best_score);
    class_ids.push_back(best_class);
  }

  std::vector<int> indices;
  if (!boxes.empty()) {
    cv::dnn::NMSBoxes(boxes, confidences, 0.25F, 0.45F, indices);
  }

  detections.reserve(indices.size());
  for (int idx : indices) {
    Detection d;
    d.class_id = class_ids[idx];
    d.confidence = confidences[idx];
    d.bbox = boxes[idx];
    detections.push_back(d);
  }

  return detections;
}

// -------------------- CrowdStatisticsNode 实现 --------------------
CrowdStatisticsNode::CrowdStatisticsNode()
    : Node("crowd_statistics_node")
{
  // 声明参数
  this->declare_parameter<std::string>("model_path", "/capella/lib/python3.10/site-packages/crowd_statistics_ws/yolov8s.onnx");
  this->declare_parameter<double>("distance_threshold", 30.0);
  this->declare_parameter<double>("confidence_threshold", 0.3);
  this->declare_parameter<double>("merge_distance", 0.5);
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<bool>("use_gpu", true);

  // 相机话题和坐标系参数（前后左右）
  this->declare_parameter<std::string>("camera_topic_front", "/rgb_camera_front/compressed");
  this->declare_parameter<std::string>("camera_topic_left", "/rgb_camera_left/compressed");
  this->declare_parameter<std::string>("camera_topic_right", "/rgb_camera_right/compressed");
  this->declare_parameter<std::string>("camera_topic_back", "/rgb_camera_back/compressed");
  this->declare_parameter<std::string>("camera_frame_front", "front_camera_color_frame");
  this->declare_parameter<std::string>("camera_frame_left", "left_camera_color_frame");
  this->declare_parameter<std::string>("camera_frame_right", "right_camera_color_frame");
  this->declare_parameter<std::string>("camera_frame_back", "back_camera_color_frame");

  // 为每个相机声明独立内参
  this->declare_parameter<double>("camera_front_fx", 454.58425);
  this->declare_parameter<double>("camera_front_fy", 452.05297);
  this->declare_parameter<double>("camera_front_cx", 309.77324);
  this->declare_parameter<double>("camera_front_cy", 280.9);

  this->declare_parameter<double>("camera_back_fx", 1312.81579);
  this->declare_parameter<double>("camera_back_fy", 1319.68694);
  this->declare_parameter<double>("camera_back_cx", 681.62665);
  this->declare_parameter<double>("camera_back_cy", 508.38222);

  this->declare_parameter<double>("camera_left_fx", 454.58425);
  this->declare_parameter<double>("camera_left_fy", 452.05297);
  this->declare_parameter<double>("camera_left_cx", 309.77324);
  this->declare_parameter<double>("camera_left_cy", 280.9);

  this->declare_parameter<double>("camera_right_fx", 454.58425);
  this->declare_parameter<double>("camera_right_fy", 452.05297);
  this->declare_parameter<double>("camera_right_cx", 309.77324);
  this->declare_parameter<double>("camera_right_cy", 280.9);

  // 获取参数
  std::string model_path = this->get_parameter("model_path").as_string();
  distance_threshold_ = this->get_parameter("distance_threshold").as_double();
  confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
  merge_distance_ = this->get_parameter("merge_distance").as_double();
  base_frame_ = this->get_parameter("base_frame").as_string();
  bool use_gpu = this->get_parameter("use_gpu").as_bool();

  // 相机配置列表（顺序与索引对应）
  std::vector<std::string> topics = {
      this->get_parameter("camera_topic_front").as_string(),
      this->get_parameter("camera_topic_left").as_string(),
      this->get_parameter("camera_topic_right").as_string(),
      this->get_parameter("camera_topic_back").as_string()
  };
  std::vector<std::string> frames = {
      this->get_parameter("camera_frame_front").as_string(),
      this->get_parameter("camera_frame_left").as_string(),
      this->get_parameter("camera_frame_right").as_string(),
      this->get_parameter("camera_frame_back").as_string()
  };
  std::vector<std::vector<double>> intrinsics = {
      {this->get_parameter("camera_front_fx").as_double(),
       this->get_parameter("camera_front_fy").as_double(),
       this->get_parameter("camera_front_cx").as_double(),
       this->get_parameter("camera_front_cy").as_double()},
      {this->get_parameter("camera_left_fx").as_double(),
       this->get_parameter("camera_left_fy").as_double(),
       this->get_parameter("camera_left_cx").as_double(),
       this->get_parameter("camera_left_cy").as_double()},
      {this->get_parameter("camera_right_fx").as_double(),
       this->get_parameter("camera_right_fy").as_double(),
       this->get_parameter("camera_right_cx").as_double(),
       this->get_parameter("camera_right_cy").as_double()},
      {this->get_parameter("camera_back_fx").as_double(),
       this->get_parameter("camera_back_fy").as_double(),
       this->get_parameter("camera_back_cx").as_double(),
       this->get_parameter("camera_back_cy").as_double()}
  };

  // 初始化 YOLO 检测器
  yolo_ = std::make_unique<YoloDetector>(model_path, this->get_logger(), use_gpu);

  // 初始化 TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 创建相机订阅器，并保存内参
  for (size_t i = 0; i < topics.size(); ++i) {
    CameraInfo cam;
    cam.topic = topics[i];
    cam.frame_id = frames[i];
    cam.fx = intrinsics[i][0];
    cam.fy = intrinsics[i][1];
    cam.cx = intrinsics[i][2];
    cam.cy = intrinsics[i][3];

    cam.sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        cam.topic,
        rclcpp::SensorDataQoS(),
        [this, i](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
          this->imageCallback(msg, i);
        });
    cameras_.push_back(std::move(cam));
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s (frame=%s, fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f)",
                topics[i].c_str(), frames[i].c_str(),
                intrinsics[i][0], intrinsics[i][1], intrinsics[i][2], intrinsics[i][3]);
  }

  // 发布器
  pub_stats_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("crowd_statistics", 10);
  pub_pedestrians_ = this->create_publisher<geometry_msgs::msg::PointStamped>("detected_pedestrians", 10);

  timer_ = this->create_wall_timer(1s, [this]() { this->timerCallback(); });

  RCLCPP_INFO(this->get_logger(), "Crowd Statistics Node initialized (4 cameras, GPU=%s)",
              use_gpu ? "enabled" : "disabled");
}

CrowdStatisticsNode::~CrowdStatisticsNode() {}

void CrowdStatisticsNode::imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, size_t cam_idx)
{
  if (!msg || msg->data.empty()) return;

  cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
  if (frame.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Failed to decode image from %s", cameras_[cam_idx].topic.c_str());
    return;
  }

  std::vector<Detection> detections = yolo_->detect(frame);
  if (detections.empty()) return;

  std::vector<geometry_msgs::msg::Point> robot_pedestrians;
  robot_pedestrians.reserve(detections.size());

  const auto &cam = cameras_[cam_idx];
  for (const auto &det : detections) {
    if (det.class_id != 0) continue;
    if (det.confidence < confidence_threshold_) continue;

    // 脚部中心
    float foot_x = det.bbox.x + det.bbox.width / 2.0f;
    float foot_y = det.bbox.y + det.bbox.height;
    cv::Point2f pixel(foot_x, foot_y);

    geometry_msgs::msg::Point robot_pt;
    if (pixelToRobotPoint(pixel, cam.frame_id, cam.fx, cam.fy, cam.cx, cam.cy, robot_pt)) {
      double dist = std::hypot(robot_pt.x, robot_pt.y);
      if (dist <= distance_threshold_) {
        robot_pedestrians.push_back(robot_pt);
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(*(cameras_[cam_idx].mutex));
    cameras_[cam_idx].latest_points = std::move(robot_pedestrians);
  }

  // 可选发布单点（可视化）
  for (const auto &pt : robot_pedestrians) {
    geometry_msgs::msg::PointStamped p_stamped;
    p_stamped.header.frame_id = base_frame_;
    p_stamped.header.stamp = msg->header.stamp;
    p_stamped.point = pt;
    pub_pedestrians_->publish(p_stamped);
  }
}

bool CrowdStatisticsNode::pixelToRobotPoint(const cv::Point2f &pixel,
                                            const std::string &camera_frame,
                                            double fx, double fy, double cx, double cy,
                                            geometry_msgs::msg::Point &robot_pt)
{
  geometry_msgs::msg::TransformStamped tf_cam_to_base;
  try {
    tf_cam_to_base = tf_buffer_->lookupTransform(base_frame_, camera_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "TF lookup failed for %s -> %s: %s", base_frame_.c_str(), camera_frame.c_str(), e.what());
    return false;
  }

  // 归一化相机坐标
  double x_cam_norm = (pixel.x - cx) / fx;
  double y_cam_norm = (pixel.y - cy) / fy;
  double z_cam_norm = 1.0;

  tf2::Vector3 d_cam(x_cam_norm, y_cam_norm, z_cam_norm);
  tf2::Vector3 p_cam_origin(0, 0, 0);

  tf2::Transform transform;
  tf2::fromMsg(tf_cam_to_base.transform, transform);
  tf2::Vector3 d_base = transform * d_cam;
  tf2::Vector3 p_base = transform * p_cam_origin;

  // 与地面平面 z=0 求交
  if (std::abs(d_base.z()) < 1e-6) return false;
  double lambda = -p_base.z() / d_base.z();
  if (lambda <= 0) return false;

  tf2::Vector3 intersection = p_base + lambda * d_base;
  robot_pt.x = intersection.x();
  robot_pt.y = intersection.y();
  robot_pt.z = 0.0;
  return true;
}

std::vector<geometry_msgs::msg::Point> CrowdStatisticsNode::mergeAndDeduplicatePoints(
    const std::vector<std::vector<geometry_msgs::msg::Point>> &all_points)
{
  std::vector<geometry_msgs::msg::Point> merged;
  for (const auto &points : all_points) {
    for (const auto &pt : points) {
      bool duplicate = false;
      for (const auto &existing : merged) {
        double dx = pt.x - existing.x;
        double dy = pt.y - existing.y;
        if (std::hypot(dx, dy) < merge_distance_) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        merged.push_back(pt);
      }
    }
  }
  return merged;
}

void CrowdStatisticsNode::timerCallback()
{
  std::vector<std::vector<geometry_msgs::msg::Point>> all_cam_points;
  for (auto &cam : cameras_) {
    std::lock_guard<std::mutex> lock(*(cam.mutex));
    if (!cam.latest_points.empty()) {
      all_cam_points.push_back(cam.latest_points);
    }
  }

  std::vector<geometry_msgs::msg::Point> unique_pedestrians = mergeAndDeduplicatePoints(all_cam_points);

  double total_people, density, pressure;
  computeStatistics(unique_pedestrians, total_people, density, pressure);

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {total_people, density, pressure};
  pub_stats_->publish(msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Crowd stats: people=%.0f, density=%.3f, pressure=%.3f",
                       total_people, density, pressure);
}

void CrowdStatisticsNode::computeStatistics(const std::vector<geometry_msgs::msg::Point> &pedestrians,
                                            double &total_people, double &density, double &pressure)
{
  total_people = static_cast<double>(pedestrians.size());
  double area = M_PI * distance_threshold_ * distance_threshold_;
  density = (area > 0) ? total_people / area : 0.0;
  pressure = density;
}

} // namespace crowd_statistics

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<crowd_statistics::CrowdStatisticsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}