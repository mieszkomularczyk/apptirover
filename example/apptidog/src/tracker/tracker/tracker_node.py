import random
import time

import numpy as np
import rclpy
from rclpy.node import Node
import shared_qos as qos
from ultralytics.engine.results import Boxes
from ultralytics.trackers import BYTETracker
from ultralytics.utils import IterableSimpleNamespace
from ultralytics.utils.checks import check_requirements

from dog_msgs.msg import DetectionStats
from foxglove_msgs.msg import (
    Color,
    ImageAnnotations,
    Point2,
    PointsAnnotation,
    TextAnnotation,
)
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2D, Detection2DArray


class TrackerNode(Node):
    """Associates YOLO detections into persistent tracks via Ultralytics' ByteTrack."""

    TRACKER_FIELDS = (
        "track_high_thresh",
        "track_low_thresh",
        "new_track_thresh",
        "track_buffer",
        "match_thresh",
        "fuse_score",
    )

    @property
    def track_high_thresh(self) -> float:
        return self.get_parameter("track_high_thresh").get_parameter_value().double_value

    @property
    def track_low_thresh(self) -> float:
        return self.get_parameter("track_low_thresh").get_parameter_value().double_value

    @property
    def new_track_thresh(self) -> float:
        return self.get_parameter("new_track_thresh").get_parameter_value().double_value

    @property
    def track_buffer(self) -> int:
        return self.get_parameter("track_buffer").get_parameter_value().integer_value

    @property
    def match_thresh(self) -> float:
        return self.get_parameter("match_thresh").get_parameter_value().double_value

    @property
    def fuse_score(self) -> bool:
        return self.get_parameter("fuse_score").get_parameter_value().bool_value

    @property
    def detections_topic(self) -> str:
        return self.get_parameter("detections_topic").get_parameter_value().string_value

    @property
    def fps_interval(self) -> float:
        return self.get_parameter("fps_interval").get_parameter_value().double_value

    def __init__(self) -> None:
        super().__init__("tracker_node")

        self.declare_parameter("track_high_thresh", 0.1)
        self.declare_parameter("track_low_thresh", 0.15)
        self.declare_parameter("new_track_thresh", 0.3)
        self.declare_parameter("track_buffer", 60)
        self.declare_parameter("match_thresh", 0.1)
        self.declare_parameter("fuse_score", True)
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("fps_interval", 5.0)

        self.tracker = self.create_tracker()
        self.class_id_map: dict[str, int] = {}
        self.color_map: dict[str, tuple] = {}
        self.enabled = False

        self.frame_count = 0
        self.total_tracking_ms = 0.0

        self.det_sub = None

        self.create_publishers()
        self.create_services()

        self.fps_timer = self.create_timer(self.fps_interval, self.fps_timer_cb)
        self.add_on_set_parameters_callback(self.parameter_callback)

    def create_publishers(self) -> None:
        self.detections_pub = self.create_publisher(
            Detection2DArray, "~/detections", qos.HIGH_FREQ_QOS
        )
        self.markers_pub = self.create_publisher(
            ImageAnnotations, "~/markers", qos.HIGH_FREQ_QOS
        )
        self.stats_pub = self.create_publisher(DetectionStats, "~/stats", qos.STATE_QOS)

    def create_services(self) -> None:
        self.enable_srv = self.create_service(SetBool, "~/enable", self.enable_cb)

    def enable_cb(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if request.data == self.enabled:
            response.success = True
            response.message = "Already enabled" if self.enabled else "Already disabled"
            return response

        self.enabled = request.data
        if self.enabled:
            self.subscribe()
            response.message = "Tracking enabled"
        else:
            self.unsubscribe()
            response.message = "Tracking disabled"
        response.success = True
        return response

    def subscribe(self, detections_topic: str | None = None) -> None:
        detections_topic = detections_topic if detections_topic is not None else self.detections_topic
        self.det_sub = self.create_subscription(
            Detection2DArray, detections_topic, self.detections_cb, qos.HIGH_FREQ_QOS
        )

    def unsubscribe(self) -> None:
        if self.det_sub is not None:
            self.destroy_subscription(self.det_sub)
            self.det_sub = None
        self.get_logger().info("Unsubscribed from topics")

        self.publish_markers([])

    def create_tracker(self, overrides: dict | None = None) -> BYTETracker:
        check_requirements("lap")
        overrides = overrides or {}
        cfg = IterableSimpleNamespace(
            tracker_type="bytetrack",
            track_high_thresh=overrides.get("track_high_thresh", self.track_high_thresh),
            track_low_thresh=overrides.get("track_low_thresh", self.track_low_thresh),
            new_track_thresh=overrides.get("new_track_thresh", self.new_track_thresh),
            track_buffer=overrides.get("track_buffer", self.track_buffer),
            match_thresh=overrides.get("match_thresh", self.match_thresh),
            fuse_score=overrides.get("fuse_score", self.fuse_score),
        )
        return BYTETracker(args=cfg)

    def detections_cb(self, detections_msg: Detection2DArray) -> None:
        tracked_msg = Detection2DArray()
        tracked_msg.header = detections_msg.header

        detection_list = []
        for detection in detections_msg.detections:
            score = detection.results[0].hypothesis.score if detection.results else 0.0
            class_id_str = detection.results[0].hypothesis.class_id if detection.results else ""
            if class_id_str not in self.class_id_map:
                self.class_id_map[class_id_str] = len(self.class_id_map)
            class_id = self.class_id_map[class_id_str]

            detection_list.append([
                detection.bbox.center.position.x - detection.bbox.size_x / 2,
                detection.bbox.center.position.y - detection.bbox.size_y / 2,
                detection.bbox.center.position.x + detection.bbox.size_x / 2,
                detection.bbox.center.position.y + detection.bbox.size_y / 2,
                score,
                class_id,
            ])

        t0 = time.monotonic()
        if detection_list:
            # orig_shape only feeds Boxes' normalized (xywhn) properties, which this
            # node never reads.
            det = Boxes(np.array(detection_list), (1, 1))
            tracks = self.tracker.update(det)

            if tracks is not None and len(tracks) > 0:
                for t in tracks:
                    tracked_box = Boxes(t[:-1], (1, 1))
                    tracked_det: Detection2D = detections_msg.detections[int(t[-1])]

                    box = tracked_box.xywh[0]
                    tracked_det.bbox.center.position.x = float(box[0])
                    tracked_det.bbox.center.position.y = float(box[1])
                    tracked_det.bbox.size_x = float(box[2])
                    tracked_det.bbox.size_y = float(box[3])

                    track_id = ""
                    if tracked_box.is_track:
                        track_id = str(int(tracked_box.id))
                    tracked_det.id = track_id

                    tracked_msg.detections.append(tracked_det)

        self.total_tracking_ms += (time.monotonic() - t0) * 1000.0
        self.frame_count += 1

        self.detections_pub.publish(tracked_msg)
        self.publish_markers(tracked_msg.detections)

    def publish_markers(self, detections: list) -> None:
        annotations = ImageAnnotations()

        for detection in detections:
            class_name = detection.results[0].hypothesis.class_id if detection.results else ""
            score = detection.results[0].hypothesis.score if detection.results else 0.0
            track_id = detection.id

            x1 = detection.bbox.center.position.x - detection.bbox.size_x / 2
            y1 = detection.bbox.center.position.y - detection.bbox.size_y / 2
            x2 = detection.bbox.center.position.x + detection.bbox.size_x / 2
            y2 = detection.bbox.center.position.y + detection.bbox.size_y / 2

            r, g, b = self.get_color_for_class(class_name)
            color = Color()
            color.r = r / 255.0
            color.g = g / 255.0
            color.b = b / 255.0
            color.a = 0.8

            tl = Point2(); tl.x = x1; tl.y = y1
            tr = Point2(); tr.x = x2; tr.y = y1
            br = Point2(); br.x = x2; br.y = y2
            bl = Point2(); bl.x = x1; bl.y = y2

            fill = Color()
            fill.r = color.r; fill.g = color.g; fill.b = color.b; fill.a = 0.2

            rect = PointsAnnotation()
            rect.type = PointsAnnotation.LINE_LOOP
            rect.points = [tl, tr, br, bl]
            rect.outline_color = color
            rect.fill_color = fill
            rect.thickness = 2.0
            annotations.points.append(rect)

            label = f"{class_name} {int(score * 100)}%"
            if track_id:
                label += f" ID:{track_id}"

            bg = Color()
            bg.r = 0.0; bg.g = 0.0; bg.b = 0.0; bg.a = 0.7

            text = TextAnnotation()
            text.position.x = x1
            text.position.y = y1
            text.text = label
            text.font_size = 12.0
            text.text_color = color
            text.background_color = bg
            annotations.texts.append(text)

        self.markers_pub.publish(annotations)

    def get_color_for_class(self, class_name: str) -> tuple:
        if class_name not in self.color_map:
            seed = self.class_id_map.get(class_name, 0)
            rng = random.Random(seed)
            self.color_map[class_name] = (
                rng.randint(77, 255),
                rng.randint(77, 255),
                rng.randint(77, 255),
            )
        return self.color_map[class_name]

    def parameter_callback(self, params: list) -> SetParametersResult:
        updates = {p.name: p.value for p in params}

        if any(field in updates for field in self.TRACKER_FIELDS):
            try:
                self.tracker = self.create_tracker(updates)
            except Exception as e:
                return SetParametersResult(successful=False, reason=str(e))
            self.get_logger().info("Tracker reconfigured")

        if "fps_interval" in updates:
            self.destroy_timer(self.fps_timer)
            self.fps_timer = self.create_timer(updates["fps_interval"], self.fps_timer_cb)

        if self.enabled and "detections_topic" in updates:
            self.unsubscribe()
            self.subscribe(detections_topic=updates["detections_topic"])

        return SetParametersResult(successful=True)

    def fps_timer_cb(self) -> None:
        msg = DetectionStats()
        msg.fps = float(self.frame_count) / float(self.fps_interval) if self.fps_interval > 0 else 0.0
        msg.inference = (
            self.total_tracking_ms / self.frame_count if self.frame_count > 0 else 0.0
        )
        self.frame_count = 0
        self.total_tracking_ms = 0.0
        self.stats_pub.publish(msg)


def main():
    rclpy.init()
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
