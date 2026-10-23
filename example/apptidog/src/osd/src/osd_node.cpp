#include "osd/osd_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace osd
{

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

OsdNode::OsdNode(const rclcpp::NodeOptions & options)
: Node("osd_node", options)
{
  // ---- Declare parameters ----
  this->declare_parameter("image_topic", "/usb_camera/image_raw");
  this->declare_parameter("is_compressed", false);
  this->declare_parameter("annotations_topics", std::vector<std::string>{"/markers"});
  this->declare_parameter("output_compressed", false);
  this->declare_parameter("jpeg_quality", 80);

  image_topic_        = this->get_parameter("image_topic").as_string();
  is_compressed_      = this->get_parameter("is_compressed").as_bool();
  annotations_topics_ = this->get_parameter("annotations_topics").as_string_array();
  output_compressed_  = this->get_parameter("output_compressed").as_bool();
  jpeg_quality_       = static_cast<int>(this->get_parameter("jpeg_quality").as_int());

  // ---- QoS – best-effort / volatile / keep-last 1 (matches shared_qos) ----
  rclcpp::QoS image_qos(rclcpp::KeepLast(1));
  image_qos.best_effort().durability_volatile();

  rclcpp::QoS annotation_qos(rclcpp::KeepLast(1));
  annotation_qos.best_effort().durability_volatile();

  // ---- Image subscription (UniquePtr for IPC zero-copy) ----
  if (is_compressed_) {
    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic_, image_qos,
      [this](sensor_msgs::msg::CompressedImage::UniquePtr msg) {
        compressed_image_cb(std::move(msg));
      });
  } else {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, image_qos,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        image_cb(std::move(msg));
      });
  }

  // ---- Annotation subscriptions ----
  for (const auto & topic : annotations_topics_) {
    auto sub = this->create_subscription<foxglove_msgs::msg::ImageAnnotations>(
      topic, annotation_qos,
      [this, topic](foxglove_msgs::msg::ImageAnnotations::SharedPtr msg) {
        annotations_cb(std::move(msg), topic);
      });
    annotations_subs_.push_back(sub);
    RCLCPP_INFO(this->get_logger(), "Subscribed to annotations topic: %s", topic.c_str());
  }

  // ---- Publisher ----
  if (output_compressed_) {
    compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "~/image_raw/compressed", image_qos);
  } else {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "~/image_raw", image_qos);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "OSD node ready  [IPC=%s]  input=%s  compressed_in=%s  compressed_out=%s",
    options.use_intra_process_comms() ? "ON" : "OFF",
    image_topic_.c_str(),
    is_compressed_ ? "true" : "false",
    output_compressed_ ? "true" : "false");
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void OsdNode::annotations_cb(
  foxglove_msgs::msg::ImageAnnotations::SharedPtr msg,
  const std::string & topic)
{
  std::lock_guard<std::mutex> lock(annotations_mutex_);
  latest_annotations_[topic] = std::move(msg);
}

void OsdNode::image_cb(sensor_msgs::msg::Image::UniquePtr msg)
{
  // Determine encoding colour order (first frame only for efficiency)
  is_bgr_ = (msg->encoding == "bgr8" || msg->encoding == "bgra8");

  const int cv_type = encoding_to_cv_type(msg->encoding);
  if (cv_type < 0) {
    // Fallback: use cv_bridge for uncommon encodings
    try {
      auto cv_ptr = cv_bridge::toCvCopy(*msg, "bgr8");
      is_bgr_ = true;
      draw_all_annotations(cv_ptr->image);
      if (output_compressed_) {
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        std::vector<uchar> buf;
        cv::imencode(".jpg", cv_ptr->image, buf, params);
        auto out = std::make_unique<sensor_msgs::msg::CompressedImage>();
        out->header = msg->header;
        out->format = "jpeg";
        out->data.assign(buf.begin(), buf.end());
        compressed_image_pub_->publish(std::move(out));
      } else {
        image_pub_->publish(*cv_ptr->toImageMsg());
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    }
    return;
  }

  // ---- Zero-copy fast path: wrap message buffer as cv::Mat, draw in-place ----
  cv::Mat image(
    static_cast<int>(msg->height),
    static_cast<int>(msg->width),
    cv_type,
    msg->data.data(),
    msg->step);

  draw_all_annotations(image);

  if (output_compressed_) {
    // Must encode → new buffer
    // Convert to BGR first if needed for JPEG encoding
    cv::Mat bgr;
    if (!is_bgr_ && image.channels() == 3) {
      cv::cvtColor(image, bgr, cv::COLOR_RGB2BGR);
    } else {
      bgr = image;  // shallow ref, no copy
    }
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    std::vector<uchar> buf;
    cv::imencode(".jpg", bgr, buf, params);
    auto out = std::make_unique<sensor_msgs::msg::CompressedImage>();
    out->header = msg->header;
    out->format = "jpeg";
    out->data.assign(buf.begin(), buf.end());
    compressed_image_pub_->publish(std::move(out));
  } else {
    // Publish the same message — zero-copy with IPC
    image_pub_->publish(std::move(msg));
  }
}

void OsdNode::compressed_image_cb(sensor_msgs::msg::CompressedImage::UniquePtr msg)
{
  // Decode compressed image
  cv::Mat raw(1, static_cast<int>(msg->data.size()), CV_8UC1, msg->data.data());
  cv::Mat image = cv::imdecode(raw, cv::IMREAD_COLOR);
  if (image.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
    return;
  }

  is_bgr_ = true;  // imdecode always returns BGR
  draw_all_annotations(image);

  if (output_compressed_) {
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    std::vector<uchar> buf;
    cv::imencode(".jpg", image, buf, params);
    auto out = std::make_unique<sensor_msgs::msg::CompressedImage>();
    out->header = msg->header;
    out->format = "jpeg";
    out->data.assign(buf.begin(), buf.end());
    compressed_image_pub_->publish(std::move(out));
  } else {
    auto out = std::make_unique<sensor_msgs::msg::Image>();
    out->header = msg->header;
    out->height = static_cast<uint32_t>(image.rows);
    out->width = static_cast<uint32_t>(image.cols);
    out->encoding = "bgr8";
    out->is_bigendian = false;
    out->step = static_cast<uint32_t>(image.step);
    out->data.assign(image.data, image.data + image.total() * image.elemSize());
    image_pub_->publish(std::move(out));
  }
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

void OsdNode::draw_all_annotations(cv::Mat & image)
{
  std::lock_guard<std::mutex> lock(annotations_mutex_);
  for (const auto & [topic, ann] : latest_annotations_) {
    draw_annotations(image, *ann);
  }
}

void OsdNode::draw_annotations(
  cv::Mat & image,
  const foxglove_msgs::msg::ImageAnnotations & annotations)
{
  for (const auto & pa : annotations.points) {
    draw_points_annotation(image, pa);
  }
  for (const auto & ta : annotations.texts) {
    draw_text_annotation(image, ta);
  }
  for (const auto & ca : annotations.circles) {
    draw_circle_annotation(image, ca);
  }
}

// ---- Points ----

void OsdNode::draw_points_annotation(
  cv::Mat & image,
  const foxglove_msgs::msg::PointsAnnotation & ann)
{
  if (ann.points.size() < 2 &&
      ann.type != foxglove_msgs::msg::PointsAnnotation::POINTS)
  {
    return;
  }

  std::vector<cv::Point> pts;
  pts.reserve(ann.points.size());
  for (const auto & p : ann.points) {
    pts.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
  }

  const cv::Scalar outline = color_to_scalar(ann.outline_color);
  const cv::Scalar fill    = color_to_scalar(ann.fill_color);
  const int thickness = ann.thickness > 0 ? static_cast<int>(ann.thickness) : 2;

  using PA = foxglove_msgs::msg::PointsAnnotation;

  switch (ann.type) {
    case PA::POINTS:
      // Per-point colours supported
      for (size_t i = 0; i < pts.size(); ++i) {
        cv::Scalar c = (i < ann.outline_colors.size())
          ? color_to_scalar(ann.outline_colors[i])
          : outline;
        cv::circle(image, pts[i], thickness, c, -1);
      }
      break;

    case PA::LINE_LOOP: {
      cv::polylines(image, pts, /*isClosed=*/true, outline, thickness);
      if (ann.fill_color.a > 0.0) {
        cv::Mat overlay;
        image.copyTo(overlay);
        cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{pts}, fill);
        double alpha = ann.fill_color.a;
        cv::addWeighted(overlay, alpha, image, 1.0 - alpha, 0.0, image);
      }
      break;
    }

    case PA::LINE_STRIP:
      // Connected line segments with optional per-point colours
      if (!ann.outline_colors.empty()) {
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
          cv::Scalar c = (i < ann.outline_colors.size())
            ? color_to_scalar(ann.outline_colors[i])
            : outline;
          cv::line(image, pts[i], pts[i + 1], c, thickness);
        }
      } else {
        cv::polylines(image, pts, /*isClosed=*/false, outline, thickness);
      }
      break;

    case PA::LINE_LIST:
      for (size_t i = 0; i + 1 < pts.size(); i += 2) {
        cv::Scalar c = outline;
        size_t ci = i / 2;
        if (ci < ann.outline_colors.size()) {
          c = color_to_scalar(ann.outline_colors[ci]);
        }
        cv::line(image, pts[i], pts[i + 1], c, thickness);
      }
      break;

    default:  // UNKNOWN – polyline
      cv::polylines(image, pts, /*isClosed=*/false, outline, thickness);
      break;
  }
}

// ---- Text ----

void OsdNode::draw_text_annotation(
  cv::Mat & image,
  const foxglove_msgs::msg::TextAnnotation & ann)
{
  const cv::Point pos(static_cast<int>(ann.position.x), static_cast<int>(ann.position.y));
  const double font_size  = ann.font_size > 0 ? ann.font_size : 12.0;
  const double font_scale = font_size / 24.0;
  const int font          = cv::FONT_HERSHEY_SIMPLEX;
  const int thick         = std::max(1, static_cast<int>(font_scale * 2));

  int baseline = 0;
  cv::Size text_sz = cv::getTextSize(ann.text, font, font_scale, thick, &baseline);

  // Background
  if (ann.background_color.a > 0.0) {
    cv::Scalar bg = color_to_scalar(ann.background_color);
    cv::Point tl(pos.x, pos.y - text_sz.height - baseline);
    cv::Point br(pos.x + text_sz.width, pos.y + baseline);
    cv::Mat overlay;
    image.copyTo(overlay);
    cv::rectangle(overlay, tl, br, bg, -1);
    double alpha = ann.background_color.a;
    cv::addWeighted(overlay, alpha, image, 1.0 - alpha, 0.0, image);
  }

  cv::Scalar tc = color_to_scalar(ann.text_color);
  cv::putText(image, ann.text, pos, font, font_scale, tc, thick, cv::LINE_AA);
}

// ---- Circle ----

void OsdNode::draw_circle_annotation(
  cv::Mat & image,
  const foxglove_msgs::msg::CircleAnnotation & ann)
{
  const cv::Point center(static_cast<int>(ann.position.x), static_cast<int>(ann.position.y));
  const int radius    = static_cast<int>(ann.diameter / 2.0);
  const int thickness = ann.thickness > 0 ? static_cast<int>(ann.thickness) : 2;

  if (ann.fill_color.a > 0.0) {
    cv::Scalar fc = color_to_scalar(ann.fill_color);
    cv::Mat overlay;
    image.copyTo(overlay);
    cv::circle(overlay, center, radius, fc, -1);
    double alpha = ann.fill_color.a;
    cv::addWeighted(overlay, alpha, image, 1.0 - alpha, 0.0, image);
  }

  cv::Scalar oc = color_to_scalar(ann.outline_color);
  cv::circle(image, center, radius, oc, thickness);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

cv::Scalar OsdNode::color_to_scalar(const foxglove_msgs::msg::Color & c) const
{
  int r = static_cast<int>(c.r * 255);
  int g = static_cast<int>(c.g * 255);
  int b = static_cast<int>(c.b * 255);
  return is_bgr_ ? cv::Scalar(b, g, r) : cv::Scalar(r, g, b);
}

int OsdNode::encoding_to_cv_type(const std::string & encoding)
{
  if (encoding == "bgr8" || encoding == "rgb8")  return CV_8UC3;
  if (encoding == "bgra8" || encoding == "rgba8") return CV_8UC4;
  if (encoding == "mono8")                        return CV_8UC1;
  if (encoding == "mono16")                       return CV_16UC1;
  if (encoding == "16UC1")                        return CV_16UC1;
  if (encoding == "32FC1")                        return CV_32FC1;
  return -1;  // unsupported → use cv_bridge fallback
}

}  // namespace osd

// ---------------------------------------------------------------------------
// Register as composable node for IPC composition
// ---------------------------------------------------------------------------
RCLCPP_COMPONENTS_REGISTER_NODE(osd::OsdNode)
