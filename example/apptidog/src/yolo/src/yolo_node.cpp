#include <atomic>
#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <dog_msgs/msg/detection_stats.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <foxglove_msgs/msg/image_annotations.hpp>
#include <foxglove_msgs/msg/points_annotation.hpp>
#include <foxglove_msgs/msg/text_annotation.hpp>
#include <foxglove_msgs/msg/color.hpp>
#include <foxglove_msgs/msg/point2.hpp>

#include "yolos/tasks/detection.hpp"

class YoloNode : public rclcpp::Node
{
public:
    explicit YoloNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("yolo_node", options)
    {
        declare_parameter("model", "");
        declare_parameter("labels", "");
        declare_parameter("image_topic", "/usb_camera/image_raw");
        declare_parameter("is_compressed", false);
        declare_parameter("threshold", 0.4);
        declare_parameter("iou", 0.7);
        declare_parameter("fps_interval", 5.0);

        image_topic_ = get_parameter("image_topic").as_string();
        is_compressed_ = get_parameter("is_compressed").as_bool();
        threshold_ = static_cast<float>(get_parameter("threshold").as_double());
        iou_ = static_cast<float>(get_parameter("iou").as_double());
        fps_interval_ = get_parameter("fps_interval").as_double();

        const auto model = get_parameter("model").as_string();
        const auto labels = get_parameter("labels").as_string();

        detector_ = std::make_unique<yolos::det::YOLODetector>(model, labels);

        rclcpp::QoS best_effort_qos(rclcpp::KeepLast(1));
        best_effort_qos.best_effort();

        detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
            "~/detections", best_effort_qos);
        markers_pub_ = create_publisher<foxglove_msgs::msg::ImageAnnotations>(
            "~/markers", best_effort_qos);
        stats_pub_ = create_publisher<dog_msgs::msg::DetectionStats>("~/stats", 10);

        enable_service_ = create_service<std_srvs::srv::SetBool>(
            "~/enable",
            [this](
                const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                enable_callback(req, res);
            });

        fps_timer_ = create_wall_timer(
            std::chrono::duration<double>(fps_interval_),
            [this]() { fps_timer_cb(); });

        RCLCPP_INFO(get_logger(), "YoloNode started (model=%s, ipc=%d)",
            model.c_str(), options.use_intra_process_comms());
    }

private:
    void subscribe()
    {
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();

        if (!is_compressed_) {
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                image_topic_, qos,
                [this](sensor_msgs::msg::Image::UniquePtr msg) {
                    image_cb(std::move(msg));
                });
        } else {
            compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                image_topic_, qos,
                [this](sensor_msgs::msg::CompressedImage::UniquePtr msg) {
                    compressed_image_cb(std::move(msg));
                });
        }

        RCLCPP_INFO(get_logger(), "Subscribed to %s", image_topic_.c_str());
    }

    void unsubscribe()
    {
        image_sub_.reset();
        compressed_sub_.reset();
        RCLCPP_INFO(get_logger(), "Unsubscribed from image topic");
    }

    void enable_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data == enabled_) {
            response->success = true;
            response->message = request->data ? "Already enabled" : "Already disabled";
            return;
        }

        enabled_ = request->data;
        if (enabled_) {
            subscribe();
            response->message = "Detection enabled";
        } else {
            unsubscribe();
            response->message = "Detection disabled";
        }
        response->success = true;
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    void image_cb(sensor_msgs::msg::Image::UniquePtr msg)
    {
        cv::Mat image;
        const auto tp0 = std::chrono::steady_clock::now();

        if (msg->encoding == "bgr8") {
            image = cv::Mat(
                msg->height, msg->width, CV_8UC3,
                const_cast<uint8_t *>(msg->data.data()), msg->step);
        } else if (msg->encoding == "rgb8") {
            cv::Mat tmp(
                msg->height, msg->width, CV_8UC3,
                const_cast<uint8_t *>(msg->data.data()), msg->step);
            cv::cvtColor(tmp, image, cv::COLOR_RGB2BGR);
        } else if (msg->encoding == "mono8") {
            cv::Mat tmp(
                msg->height, msg->width, CV_8UC1,
                const_cast<uint8_t *>(msg->data.data()), msg->step);
            cv::cvtColor(tmp, image, cv::COLOR_GRAY2BGR);
        } else if (msg->encoding == "yuv422_yuy2" || msg->encoding == "yuv422") {
            cv::Mat tmp(
                msg->height, msg->width, CV_8UC2,
                const_cast<uint8_t *>(msg->data.data()), msg->step);
            cv::cvtColor(tmp, image, cv::COLOR_YUV2BGR_YUYV);
        } else {
            RCLCPP_WARN_ONCE(get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }

        total_preprocess_ms_ += std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - tp0).count();

        run_detection(image, msg->header);
    }

    void compressed_image_cb(sensor_msgs::msg::CompressedImage::UniquePtr msg)
    {
        const auto tp0 = std::chrono::steady_clock::now();
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        total_preprocess_ms_ += std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - tp0).count();

        if (image.empty()) {
            RCLCPP_ERROR(get_logger(), "Failed to decode compressed image");
            return;
        }
        run_detection(image, msg->header);
    }

    void run_detection(const cv::Mat & image, const std_msgs::msg::Header & header)
    {
        const auto t0 = std::chrono::steady_clock::now();
        auto detections = detector_->detect(image, threshold_, iou_);
        const auto t1 = std::chrono::steady_clock::now();
        total_inference_ms_ += std::chrono::duration<float, std::milli>(t1 - t0).count();
        frame_count_++;
        publish_detections(detections, header);
        publish_markers(detections);
    }

    void publish_detections(
        const std::vector<yolos::det::Detection> & detections,
        const std_msgs::msg::Header & header)
    {
        vision_msgs::msg::Detection2DArray array;
        array.header = header;

        const auto & names = detector_->getClassNames();

        for (const auto & det : detections) {
            vision_msgs::msg::Detection2D d;
            d.bbox.center.position.x = det.box.x + det.box.width / 2.0;
            d.bbox.center.position.y = det.box.y + det.box.height / 2.0;
            d.bbox.size_x = static_cast<double>(det.box.width);
            d.bbox.size_y = static_cast<double>(det.box.height);

            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            if (det.classId >= 0 && static_cast<size_t>(det.classId) < names.size()) {
                hyp.hypothesis.class_id = names[det.classId];
            }
            hyp.hypothesis.score = det.conf;
            d.results.push_back(hyp);
            array.detections.push_back(d);
        }

        detections_pub_->publish(array);
    }

    void publish_markers(const std::vector<yolos::det::Detection> & detections)
    {
        foxglove_msgs::msg::ImageAnnotations annotations;
        const auto & names = detector_->getClassNames();

        for (const auto & det : detections) {
            if (det.classId < 0 || static_cast<size_t>(det.classId) >= names.size()) {
                continue;
            }

            const double x1 = det.box.x;
            const double y1 = det.box.y;
            const double x2 = det.box.x + det.box.width;
            const double y2 = det.box.y + det.box.height;

            const cv::Scalar cv_color = get_color_for_class(det.classId);
            foxglove_msgs::msg::Color color;
            color.r = static_cast<float>(cv_color[2]) / 255.0f;
            color.g = static_cast<float>(cv_color[1]) / 255.0f;
            color.b = static_cast<float>(cv_color[0]) / 255.0f;
            color.a = 0.8f;

            foxglove_msgs::msg::PointsAnnotation rect;
            rect.type = foxglove_msgs::msg::PointsAnnotation::LINE_LOOP;
            foxglove_msgs::msg::Point2 tl, tr, br, bl;
            tl.x = x1; tl.y = y1;
            tr.x = x2; tr.y = y1;
            br.x = x2; br.y = y2;
            bl.x = x1; bl.y = y2;
            rect.points = {tl, tr, br, bl};
            rect.outline_color = color;
            foxglove_msgs::msg::Color fill = color;
            fill.a = 0.2f;
            rect.fill_color = fill;
            rect.thickness = 2.0f;
            annotations.points.push_back(rect);

            foxglove_msgs::msg::TextAnnotation text;
            text.position.x = x1;
            text.position.y = y1;
            text.text = names[det.classId] + " " +
                        std::to_string(static_cast<int>(det.conf * 100)) + "%";
            text.font_size = 12.0f;
            text.text_color = color;
            foxglove_msgs::msg::Color bg;
            bg.r = 0.0f; bg.g = 0.0f; bg.b = 0.0f; bg.a = 0.7f;
            text.background_color = bg;
            annotations.texts.push_back(text);
        }

        markers_pub_->publish(annotations);
    }

    void fps_timer_cb()
    {
        dog_msgs::msg::DetectionStats msg;
        msg.fps = static_cast<float>(frame_count_) / static_cast<float>(fps_interval_);
        msg.inference = (frame_count_ > 0) ? (total_inference_ms_ / static_cast<float>(frame_count_)) : 0.0f;
        msg.preprocess = (frame_count_ > 0) ? (total_preprocess_ms_ / static_cast<float>(frame_count_)) : 0.0f;
        frame_count_ = 0;
        total_inference_ms_ = 0.0f;
        total_preprocess_ms_ = 0.0f;
        stats_pub_->publish(msg);
    }

    cv::Scalar get_color_for_class(int class_id)
    {
        auto it = colors_.find(class_id);
        if (it != colors_.end()) {
            return it->second;
        }
        std::mt19937 rng(static_cast<uint32_t>(class_id));
        std::uniform_int_distribution<int> dist(77, 255);
        cv::Scalar color(dist(rng), dist(rng), dist(rng));
        colors_[class_id] = color;
        return color;
    }

    std::string image_topic_;
    bool is_compressed_;
    float threshold_;
    float iou_;
    double fps_interval_;
    bool enabled_{false};
    int frame_count_{0};
    float total_inference_ms_{0.0f};
    float total_preprocess_ms_{0.0f};

    std::unique_ptr<yolos::det::YOLODetector> detector_;
    std::unordered_map<int, cv::Scalar> colors_;

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
    rclcpp::Publisher<foxglove_msgs::msg::ImageAnnotations>::SharedPtr markers_pub_;
    rclcpp::Publisher<dog_msgs::msg::DetectionStats>::SharedPtr stats_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    rclcpp::TimerBase::SharedPtr fps_timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(YoloNode)

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    auto node = std::make_shared<YoloNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
