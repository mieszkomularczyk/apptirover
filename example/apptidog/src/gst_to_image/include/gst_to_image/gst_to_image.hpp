#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

// Receives a video stream via a GStreamer pipeline (ending in an appsink named
// 'imagesink') and republishes each frame on ~/image_raw, plus a JPEG-encoded
// ~/image_raw/compressed while that topic has subscribers. The inverse of the
// image_to_gst node.
class GstToImage : public rclcpp::Node
{
public:
  explicit GstToImage(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GstToImage();

private:
  void create_publishers();
  bool create_pipeline();
  void gst_mainloop_start();
  void gst_mainloop_stop();

  void start_streaming();

  // Called on the GStreamer streaming thread for every decoded frame.
  GstFlowReturn on_new_sample(GstAppSink * appsink);
  static GstFlowReturn on_new_sample_static(GstAppSink * appsink, gpointer user_data);

  // Surfaces GStreamer bus errors (e.g. caps negotiation failure, device busy)
  // that would otherwise leave the pipeline silently producing no frames.
  gboolean on_bus_message(GstMessage * message);
  static gboolean on_bus_message_static(GstBus * bus, GstMessage * message, gpointer user_data);

  // JPEG encoding runs off the GStreamer thread so a slow encode never delays
  // ~/image_raw. Only the latest frame is kept; encoding never falls behind.
  void compression_thread_start();
  void compression_thread_stop();
  void compression_thread_run();
  void publish_compressed(const sensor_msgs::msg::Image & frame);
  static int encoding_to_cv_type(const std::string & encoding);

  std::string pipeline_str_;
  std::string frame_id_;
  int jpeg_quality_;

  GstElement * pipeline_;
  GstAppSink * appsink_;
  GMainLoop * loop_;
  std::thread gst_thread_;
  guint bus_watch_id_{0};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;

  std::thread compression_thread_;
  std::mutex compression_mutex_;
  std::condition_variable compression_cv_;
  sensor_msgs::msg::Image::UniquePtr pending_frame_;
  bool compression_stop_{false};
};
