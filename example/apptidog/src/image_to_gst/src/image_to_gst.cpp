#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "../include/image_to_gst.hpp"

// Image encodings (from sensor_msgs)
namespace sensor_msgs {
namespace image_encodings {
    const std::string RGB8 = "rgb8";
    const std::string RGB16 = "rgb16";
    const std::string RGBA8 = "rgba8";
    const std::string RGBA16 = "rgba16";
    const std::string BGR8 = "bgr8";
    const std::string BGR16 = "bgr16";
    const std::string BGRA8 = "bgra8";
    const std::string BGRA16 = "bgra16";
    const std::string MONO8 = "mono8";
    const std::string MONO16 = "mono16";
    const std::string YUV422 = "yuv422";
}
}

ImageToGst::ImageToGst(const rclcpp::NodeOptions & options)
: Node("image_to_gst", options), pipeline_(nullptr), appsrc_(nullptr), loop_(nullptr) {
    // Declare and get the parameters
    this->declare_parameter("topic", "/color/image_raw");
    this->declare_parameter("compressed", false);
    this->declare_parameter("pipeline", R"(
        appsrc name=imagesrc do-timestamp=true is-live=true format=time !
        queue max-size-buffers=1 leaky=downstream !
        videoconvert !
        autovideosink
    )");

    topic_ = this->get_parameter("topic").as_string();
    compressed_ = this->get_parameter("compressed").as_bool();
    pipeline_str_ = this->get_parameter("pipeline").as_string();

    // Replace <IP> placeholder with VIDEO_DESTINATION_IP environment variable
    const char *video_destination_ip_env = std::getenv("VIDEO_DESTINATION_IP");
    std::string video_destination_ip = video_destination_ip_env ? video_destination_ip_env : "127.0.0.1";
    {
        std::string placeholder = "<IP>";
        size_t pos = 0;
        while ((pos = pipeline_str_.find(placeholder, pos)) != std::string::npos) {
            pipeline_str_.replace(pos, placeholder.length(), video_destination_ip);
            pos += video_destination_ip.length();
        }
    }

    framerate_ = extract_framerate(pipeline_str_, 30);

    // Initialize GStreamer
    gst_init(NULL, NULL);

    // Create the pipeline
    if (!create_pipeline()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline");
        return;
    }

    // Start the GStreamer main loop
    gst_mainloop_start();

    // Start the subscription
    subscribe();
}

ImageToGst::~ImageToGst() {
    gst_mainloop_stop();
    
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

void ImageToGst::subscribe() {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort().durability_volatile();

    if (!compressed_) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_, qos,
            [this](sensor_msgs::msg::Image::UniquePtr msg) {
                this->topic_callback(std::move(msg));
            });
    } else {
        subscription_compressed_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            topic_, qos,
            [this](sensor_msgs::msg::CompressedImage::UniquePtr msg) {
                this->compressed_topic_callback(std::move(msg));
            });
    }
}

bool ImageToGst::create_pipeline() {
    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str_.c_str(), &error);
    
    if (error) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse pipeline: %s", error->message);
        g_error_free(error);
        return false;
    }

    if (!pipeline_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline");
        return false;
    }

    // Get the appsrc element
    GstElement *appsrc_element = gst_bin_get_by_name(GST_BIN(pipeline_), "imagesrc");
    if (!appsrc_element) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get appsrc element 'imagesrc' from pipeline");
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }
    appsrc_ = GST_APP_SRC(appsrc_element);

    // Configure appsrc
    gst_util_set_object_arg(G_OBJECT(appsrc_), "format", "time");

    // Set pipeline to playing state
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set pipeline to PLAYING state");
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }

    return true;
}

void ImageToGst::gst_mainloop_start() {
    loop_ = g_main_loop_new(NULL, FALSE);
    gst_thread_ = std::thread([this]() {
        g_main_loop_run(loop_);
    });
}

void ImageToGst::gst_mainloop_stop() {
    if (loop_) {
        g_main_loop_quit(loop_);
        if (gst_thread_.joinable()) {
            gst_thread_.join();
        }
        g_main_loop_unref(loop_);
        loop_ = nullptr;
    }
}

uint ImageToGst::extract_framerate(const std::string& pipeline, uint default_framerate) {
    std::string search_str = "framerate=";
    size_t pos = pipeline.find(search_str);
    if (pos == std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "Framerate not found in pipeline, using default: %d", default_framerate);
        return default_framerate;
    }

    pos += search_str.length();

    size_t end_pos = pipeline.find_first_of("/,", pos);
    if (end_pos == std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "Invalid framerate format in pipeline, using default: %d", default_framerate);
        return default_framerate;
    }

    std::string framerate_str = pipeline.substr(pos, end_pos - pos);
    framerate_str.erase(0, framerate_str.find_first_not_of(" \t"));
    framerate_str.erase(framerate_str.find_last_not_of(" \t") + 1);
    
    try {
        uint framerate = std::stoi(framerate_str);
        if (framerate <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid framerate value %d, using default: %d", framerate, default_framerate);
            return default_framerate;
        }
        return framerate;
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse framerate '%s', using default: %d", framerate_str.c_str(), default_framerate);
        return default_framerate;
    }
}

GstCaps *ImageToGst::gst_caps_new_from_image(const sensor_msgs::msg::Image & msg) {
    static const std::map<std::string, std::string> known_formats = {
        {sensor_msgs::image_encodings::RGB8, "RGB"},
        {sensor_msgs::image_encodings::RGB16, "RGB16"},
        {sensor_msgs::image_encodings::RGBA8, "RGBA"},
        {sensor_msgs::image_encodings::RGBA16, "RGBA16"},
        {sensor_msgs::image_encodings::BGR8, "BGR"},
        {sensor_msgs::image_encodings::BGR16, "BGR16"},
        {sensor_msgs::image_encodings::BGRA8, "BGRA"},
        {sensor_msgs::image_encodings::BGRA16, "BGRA16"},
        {sensor_msgs::image_encodings::MONO8, "GRAY8"},
        {sensor_msgs::image_encodings::MONO16, "GRAY16_LE"},
        {sensor_msgs::image_encodings::YUV422, "YUY2"},
    };

    if (msg.is_bigendian) {
        RCLCPP_ERROR(this->get_logger(), "GST: big endian image format is not supported");
        return nullptr;
    }

    auto format = known_formats.find(msg.encoding);
    if (format == known_formats.end()) {
        RCLCPP_ERROR(this->get_logger(), "GST: image format '%s' unknown", msg.encoding.c_str());
        return nullptr;
    }

    return gst_caps_new_simple("video/x-raw",
                               "format", G_TYPE_STRING, format->second.c_str(),
                               "width", G_TYPE_INT, msg.width,
                               "height", G_TYPE_INT, msg.height,
                               "framerate", GST_TYPE_FRACTION, framerate_, 1,
                               nullptr);
}

void ImageToGst::update_caps_if_needed(const sensor_msgs::msg::Image & msg) {
    if (msg.encoding == last_encoding_ && msg.width == last_width_ && msg.height == last_height_) {
        return;  // caps unchanged, skip renegotiation
    }

    GstCaps *caps = gst_caps_new_from_image(msg);
    if (!caps) return;

    gst_app_src_set_caps(appsrc_, caps);
    gst_caps_unref(caps);

    last_encoding_ = msg.encoding;
    last_width_ = msg.width;
    last_height_ = msg.height;

    RCLCPP_INFO(this->get_logger(), "Updated caps: %s %dx%d", msg.encoding.c_str(), msg.width, msg.height);
}

void ImageToGst::topic_callback(sensor_msgs::msg::Image::UniquePtr msg) {
    if (appsrc_ == nullptr) return;

    // Only renegotiate caps when format/size actually changes
    update_caps_if_needed(*msg);

    // Create buffer and push
    GstBuffer *buf = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
    gst_buffer_fill(buf, 0, msg->data.data(), msg->data.size());
    GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);
    
    GstFlowReturn ret = gst_app_src_push_buffer(appsrc_, buf);
    if (ret != GST_FLOW_OK) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "Failed to push buffer to GStreamer pipeline");
    }
}

void ImageToGst::compressed_topic_callback(sensor_msgs::msg::CompressedImage::UniquePtr msg) {
    if (appsrc_ == nullptr) return;

    // Decompress the image
    cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);
    if (img.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to decompress image");
        return;
    }

    // Determine the GStreamer caps
    std::string gst_format;
    switch (img.type()) {
        case CV_8UC3: gst_format = "BGR"; break;
        case CV_8UC4: gst_format = "RGBA"; break;
        case CV_8UC1: gst_format = "GRAY8"; break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unsupported image type");
            return;
    }

    GstCaps *caps = gst_caps_new_simple("video/x-raw",
                                        "format", G_TYPE_STRING, gst_format.c_str(),
                                        "width", G_TYPE_INT, img.cols,
                                        "height", G_TYPE_INT, img.rows,
                                        "framerate", GST_TYPE_FRACTION, framerate_, 1,
                                        nullptr);

    gst_app_src_set_caps(appsrc_, caps);
    gst_caps_unref(caps);

    // Create a GstBuffer and fill it with the image data
    GstBuffer *buf = gst_buffer_new_allocate(nullptr, img.total() * img.elemSize(), nullptr);
    gst_buffer_fill(buf, 0, img.data, img.total() * img.elemSize());
    GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);

    // Push the buffer to GStreamer
    GstFlowReturn ret = gst_app_src_push_buffer(appsrc_, buf);
    if (ret != GST_FLOW_OK) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "Failed to push buffer to GStreamer pipeline");
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImageToGst)

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    auto node = std::make_shared<ImageToGst>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
