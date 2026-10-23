#pragma once

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <foxglove_msgs/msg/image_annotations.hpp>
#include <foxglove_msgs/msg/points_annotation.hpp>
#include <foxglove_msgs/msg/text_annotation.hpp>
#include <foxglove_msgs/msg/circle_annotation.hpp>
#include <foxglove_msgs/msg/color.hpp>
#include <foxglove_msgs/msg/point2.hpp>
#include <opencv2/opencv.hpp>

namespace osd
{

class OsdNode : public rclcpp::Node
{
public:
  explicit OsdNode(const rclcpp::NodeOptions & options);

private:
  // ---------- Parameters ----------
  std::string image_topic_;
  bool is_compressed_;
  std::vector<std::string> annotations_topics_;
  bool output_compressed_;
  int jpeg_quality_;

  // ---------- ROS I/O ----------
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  std::vector<rclcpp::Subscription<foxglove_msgs::msg::ImageAnnotations>::SharedPtr>
    annotations_subs_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;

  // ---------- State ----------
  std::map<std::string, foxglove_msgs::msg::ImageAnnotations::SharedPtr> latest_annotations_;
  std::mutex annotations_mutex_;

  /// True when the input image encoding is BGR-ordered (bgr8 / bgra8).
  bool is_bgr_{true};

  // ---------- Callbacks ----------
  void image_cb(sensor_msgs::msg::Image::UniquePtr msg);
  void compressed_image_cb(sensor_msgs::msg::CompressedImage::UniquePtr msg);
  void annotations_cb(
    foxglove_msgs::msg::ImageAnnotations::SharedPtr msg,
    const std::string & topic);

  // ---------- Drawing ----------
  void draw_all_annotations(cv::Mat & image);
  void draw_annotations(
    cv::Mat & image,
    const foxglove_msgs::msg::ImageAnnotations & annotations);
  void draw_points_annotation(
    cv::Mat & image,
    const foxglove_msgs::msg::PointsAnnotation & ann);
  void draw_text_annotation(
    cv::Mat & image,
    const foxglove_msgs::msg::TextAnnotation & ann);
  void draw_circle_annotation(
    cv::Mat & image,
    const foxglove_msgs::msg::CircleAnnotation & ann);

  /// Convert a foxglove Color (RGBA 0-1) to a cv::Scalar respecting current
  /// encoding order (BGR or RGB).
  cv::Scalar color_to_scalar(const foxglove_msgs::msg::Color & c) const;

  // ---------- Helpers ----------
  static int encoding_to_cv_type(const std::string & encoding);
};

}  // namespace osd
