#ifndef CROWD_STATISTICS__CROWD_STATISTICS_HPP_
#define CROWD_STATISTICS__CROWD_STATISTICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <memory>
#include <vector>
#include <deque>
#include <mutex>
#include <string>

namespace crowd_statistics
{

struct Detection
{
  int class_id;
  float confidence;
  cv::Rect bbox;
};

class YoloDetector
{
public:
  YoloDetector(const std::string &model_path, const rclcpp::Logger &logger, bool use_gpu = true);
  std::vector<Detection> detect(const cv::Mat &frame);

private:
  rclcpp::Logger logger_;
  cv::dnn::Net net_;
  bool enabled_;
  bool use_gpu_;
};

// 相机信息结构体（可移动）
struct CameraInfo
{
  std::string topic;
  std::string frame_id;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
  std::vector<geometry_msgs::msg::Point> latest_points;
  std::unique_ptr<std::mutex> mutex;

  // 内参
  double fx, fy, cx, cy;

  CameraInfo() : mutex(std::make_unique<std::mutex>()) {}
  CameraInfo(CameraInfo &&) = default;
  CameraInfo &operator=(CameraInfo &&) = default;
  CameraInfo(const CameraInfo &) = delete;
  CameraInfo &operator=(const CameraInfo &) = delete;
};

class CrowdStatisticsNode : public rclcpp::Node
{
public:
  CrowdStatisticsNode();
  ~CrowdStatisticsNode();

private:
  std::vector<CameraInfo> cameras_;
  std::unique_ptr<YoloDetector> yolo_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_stats_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_pedestrians_;

  rclcpp::TimerBase::SharedPtr timer_;

  double distance_threshold_;
  double confidence_threshold_;
  double merge_distance_;
  std::string base_frame_;

  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, size_t cam_idx);
  bool pixelToRobotPoint(const cv::Point2f &pixel,
                         const std::string &camera_frame,
                         double fx, double fy, double cx, double cy,
                         geometry_msgs::msg::Point &robot_pt);
  std::vector<geometry_msgs::msg::Point> mergeAndDeduplicatePoints(
      const std::vector<std::vector<geometry_msgs::msg::Point>> &all_points);
  void computeStatistics(const std::vector<geometry_msgs::msg::Point> &pedestrians,
                         double &total_people, double &density, double &pressure);
  void timerCallback();
};

} // namespace crowd_statistics

#endif // CROWD_STATISTICS__CROWD_STATISTICS_HPP_