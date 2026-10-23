#ifndef IMAGE_TO_GST_IMAGE_TO_GST_HPP
#define IMAGE_TO_GST_IMAGE_TO_GST_HPP

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>

class ImageToGst : public rclcpp::Node {
public:
    explicit ImageToGst(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~ImageToGst();

private:
    std::string topic_;
    std::string pipeline_str_;
    uint framerate_;
    bool compressed_;
    
    GstElement *pipeline_;
    GstAppSrc *appsrc_;
    GMainLoop *loop_;
    std::thread gst_thread_;

    void gst_mainloop_start();
    void gst_mainloop_stop();
    bool create_pipeline();
    void subscribe();
    void topic_callback(sensor_msgs::msg::Image::UniquePtr msg);
    void compressed_topic_callback(sensor_msgs::msg::CompressedImage::UniquePtr msg);
    uint extract_framerate(const std::string& pipeline, uint default_framerate);
    GstCaps *gst_caps_new_from_image(const sensor_msgs::msg::Image & msg);
    void update_caps_if_needed(const sensor_msgs::msg::Image & msg);

    // Cached caps state to avoid renegotiation every frame
    std::string last_encoding_;
    uint32_t last_width_{0};
    uint32_t last_height_{0};
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_compressed_;
};

#endif // IMAGE_TO_GST_IMAGE_TO_GST_HPP
