#include "gst_to_image/gst_to_image.hpp"

#include <cstdlib>
#include <map>

#include <rclcpp_components/register_node_macro.hpp>

namespace
{
// GStreamer video/x-raw 'format' -> ROS sensor_msgs image encoding.
const std::map<std::string, std::string> & gst_to_ros_encodings()
{
  static const std::map<std::string, std::string> map = {
    {"RGB", "rgb8"},
    {"BGR", "bgr8"},
    {"RGBA", "rgba8"},
    {"BGRA", "bgra8"},
    {"GRAY8", "mono8"},
    {"GRAY16_LE", "mono16"},
    {"RGB16", "rgb16"},
    {"BGR16", "bgr16"},
    {"YUY2", "yuv422"},
  };
  return map;
}
}  // namespace

GstToImage::GstToImage(const rclcpp::NodeOptions & options)
: Node("gst_to_image", options),
  pipeline_(nullptr), appsink_(nullptr), loop_(nullptr)
{
  // A receive pipeline must terminate in an appsink named 'imagesink'.
  declare_parameter("pipeline", R"(
      udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" !
      rtph264depay ! h264parse ! avdec_h264 ! videoconvert !
      video/x-raw,format=BGR !
      appsink name=imagesink
  )");
  declare_parameter("frame_id", "camera");
  declare_parameter("jpeg_quality", 80);

  pipeline_str_ = get_parameter("pipeline").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  jpeg_quality_ = static_cast<int>(get_parameter("jpeg_quality").as_int());

  // Mirror image_to_gst: allow a <IP> placeholder resolved from the environment.
  const char * video_destination_ip_env = std::getenv("VIDEO_DESTINATION_IP");
  std::string video_destination_ip = video_destination_ip_env ? video_destination_ip_env : "127.0.0.1";
  for (size_t pos = 0; (pos = pipeline_str_.find("<IP>", pos)) != std::string::npos;) {
    pipeline_str_.replace(pos, 4, video_destination_ip);
    pos += video_destination_ip.length();
  }

  create_publishers();
  compression_thread_start();

  gst_init(nullptr, nullptr);
  if (!create_pipeline()) {
    RCLCPP_ERROR(get_logger(), "Failed to create GStreamer pipeline");
    return;
  }
  gst_mainloop_start();

  start_streaming();

  RCLCPP_INFO(get_logger(), "GstToImage started");
}

GstToImage::~GstToImage()
{
  gst_mainloop_stop();
  compression_thread_stop();
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
}

void GstToImage::create_publishers()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort().durability_volatile();
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", qos);
  compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    "~/image_raw/compressed", qos);
}

bool GstToImage::create_pipeline()
{
  GError * error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_str_.c_str(), &error);
  if (error) {
    RCLCPP_ERROR(get_logger(), "Failed to parse pipeline: %s", error->message);
    g_error_free(error);
    return false;
  }
  if (!pipeline_) {
    RCLCPP_ERROR(get_logger(), "Failed to create pipeline");
    return false;
  }

  GstElement * appsink_element = gst_bin_get_by_name(GST_BIN(pipeline_), "imagesink");
  if (!appsink_element) {
    RCLCPP_ERROR(get_logger(), "Failed to get appsink element 'imagesink' from pipeline");
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return false;
  }
  appsink_ = GST_APP_SINK(appsink_element);

  // Pull the freshest frame only; never block the streaming thread on ROS.
  gst_app_sink_set_emit_signals(appsink_, TRUE);
  gst_app_sink_set_max_buffers(appsink_, 1);
  gst_app_sink_set_drop(appsink_, TRUE);
  g_object_set(G_OBJECT(appsink_), "sync", FALSE, nullptr);
  g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample_static), this);

  GstBus * bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
  bus_watch_id_ = gst_bus_add_watch(bus, on_bus_message_static, this);
  gst_object_unref(bus);

  return true;
}

void GstToImage::gst_mainloop_start()
{
  loop_ = g_main_loop_new(nullptr, FALSE);
  gst_thread_ = std::thread([this]() { g_main_loop_run(loop_); });
}

void GstToImage::gst_mainloop_stop()
{
  if (loop_) {
    g_main_loop_quit(loop_);
    if (gst_thread_.joinable()) {
      gst_thread_.join();
    }
    g_main_loop_unref(loop_);
    loop_ = nullptr;
  }
  if (bus_watch_id_ != 0) {
    g_source_remove(bus_watch_id_);
    bus_watch_id_ = 0;
  }
}

void GstToImage::compression_thread_start()
{
  compression_thread_ = std::thread([this]() { compression_thread_run(); });
}

void GstToImage::compression_thread_stop()
{
  {
    std::lock_guard<std::mutex> lock(compression_mutex_);
    compression_stop_ = true;
  }
  compression_cv_.notify_one();
  if (compression_thread_.joinable()) {
    compression_thread_.join();
  }
}

void GstToImage::compression_thread_run()
{
  while (true) {
    std::unique_lock<std::mutex> lock(compression_mutex_);
    compression_cv_.wait(
      lock, [this]() { return pending_frame_ != nullptr || compression_stop_; });
    if (!pending_frame_) {
      return;
    }
    auto frame = std::move(pending_frame_);
    lock.unlock();

    publish_compressed(*frame);
  }
}

void GstToImage::publish_compressed(const sensor_msgs::msg::Image & frame)
{
  const int cv_type = encoding_to_cv_type(frame.encoding);
  if (cv_type < 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cannot JPEG-encode unsupported format '%s'", frame.encoding.c_str());
    return;
  }

  cv::Mat image(
    static_cast<int>(frame.height),
    static_cast<int>(frame.width),
    cv_type,
    const_cast<uint8_t *>(frame.data.data()),
    frame.step);

  cv::Mat bgr;
  const bool is_rgb_order = frame.encoding == "rgb8" || frame.encoding == "rgba8";
  if (is_rgb_order) {
    cv::cvtColor(image, bgr, image.channels() == 4 ? cv::COLOR_RGBA2BGR : cv::COLOR_RGB2BGR);
  } else {
    bgr = image;
  }

  auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
  compressed_msg->header = frame.header;
  compressed_msg->format = "jpeg";
  std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
  cv::imencode(".jpg", bgr, compressed_msg->data, jpeg_params);
  compressed_pub_->publish(std::move(compressed_msg));
}

int GstToImage::encoding_to_cv_type(const std::string & encoding)
{
  if (encoding == "bgr8" || encoding == "rgb8") {return CV_8UC3;}
  if (encoding == "bgra8" || encoding == "rgba8") {return CV_8UC4;}
  if (encoding == "mono8") {return CV_8UC1;}
  if (encoding == "mono16") {return CV_16UC1;}
  return -1;
}

void GstToImage::start_streaming()
{
  if (!pipeline_) {
    return;
  }
  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Failed to transition pipeline to PLAYING");
    return;
  }
  RCLCPP_INFO(get_logger(), "Streaming started");
}

GstFlowReturn GstToImage::on_new_sample_static(GstAppSink * appsink, gpointer user_data)
{
  return static_cast<GstToImage *>(user_data)->on_new_sample(appsink);
}

gboolean GstToImage::on_bus_message_static(GstBus *, GstMessage * message, gpointer user_data)
{
  return static_cast<GstToImage *>(user_data)->on_bus_message(message);
}

gboolean GstToImage::on_bus_message(GstMessage * message)
{
  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR: {
      GError * error = nullptr;
      gchar * debug = nullptr;
      gst_message_parse_error(message, &error, &debug);
      RCLCPP_ERROR(
        get_logger(), "GStreamer pipeline error: %s (%s)",
        error->message, debug ? debug : "no debug info");
      g_clear_error(&error);
      g_free(debug);
      break;
    }
    case GST_MESSAGE_EOS:
      RCLCPP_WARN(get_logger(), "GStreamer pipeline reached end-of-stream");
      break;
    default:
      break;
  }
  return TRUE;
}

GstFlowReturn GstToImage::on_new_sample(GstAppSink * appsink)
{
  GstSample * sample = gst_app_sink_pull_sample(appsink);
  if (!sample) {
    return GST_FLOW_OK;
  }

  GstCaps * caps = gst_sample_get_caps(sample);
  GstStructure * structure = caps ? gst_caps_get_structure(caps, 0) : nullptr;
  const char * format = structure ? gst_structure_get_string(structure, "format") : nullptr;
  int width = 0;
  int height = 0;
  if (!structure ||
    !gst_structure_get_int(structure, "width", &width) ||
    !gst_structure_get_int(structure, "height", &height) ||
    format == nullptr)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Sample missing video caps");
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  const auto & encodings = gst_to_ros_encodings();
  auto encoding = encodings.find(format);
  if (encoding == encodings.end()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Unsupported GStreamer format '%s'", format);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstMapInfo info;
  if (!buffer || !gst_buffer_map(buffer, &info, GST_MAP_READ)) {
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  // New unique_ptr per frame so publishes use IPC zero-copy move semantics.
  auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
  image_msg->header.stamp = now();
  image_msg->header.frame_id = frame_id_;
  image_msg->width = static_cast<uint32_t>(width);
  image_msg->height = static_cast<uint32_t>(height);
  image_msg->encoding = encoding->second;
  image_msg->is_bigendian = false;
  image_msg->step = static_cast<uint32_t>(info.size / height);
  image_msg->data.assign(info.data, info.data + info.size);

  gst_buffer_unmap(buffer, &info);
  gst_sample_unref(sample);

  if (compressed_pub_->get_subscription_count() > 0) {
    std::lock_guard<std::mutex> lock(compression_mutex_);
    pending_frame_ = std::make_unique<sensor_msgs::msg::Image>(*image_msg);
    compression_cv_.notify_one();
  }

  image_pub_->publish(std::move(image_msg));
  return GST_FLOW_OK;
}

RCLCPP_COMPONENTS_REGISTER_NODE(GstToImage)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<GstToImage>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
