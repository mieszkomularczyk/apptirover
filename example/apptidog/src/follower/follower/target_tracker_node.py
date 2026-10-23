#!/usr/bin/env python3
"""
Target Tracker Node

This node selects which tracked person to follow, filters their bounding box,
and publishes the normalized target bbox and visibility state for the control
systems downstream.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import shared_qos as qos

from typing import Optional
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from dog_msgs.msg import Bbox2D
from foxglove_msgs.msg import ImageAnnotations, PointsAnnotation, Point2

from .bbox_estimator import BBoxEstimator
from .target_selector import Candidate, TargetSelector

class TargetTrackerNode(Node):
    """
    Main target tracker node that calculates offset from desired target position.
    
    This node receives tracked detections, selects the target to follow, and
    publishes the filtered normalized bbox for control systems.
    """

    def __init__(self):
        super().__init__('target_tracker_node')
        
        # Parameters
        # Upstream topics/services (override per camera -- see visual_guidance.yaml)
        self.declare_parameter('detections_topic', '/tracker/detections')
        self.declare_parameter('image_topic', '/usb_camera/image_raw')
        self.declare_parameter('yolo_enable_service', '/yolo/enable')
        self.declare_parameter('tracker_enable_service', '/tracker/enable')

        self.declare_parameter('visible_window_sec', 1.0)  # Time window to check for visibility
        self.declare_parameter('visible_threshold_percent', 0.8)  # Percentage of detections with target to be considered visible
        self.declare_parameter('invisible_window_sec', 1.0)  # Time window to check for invisibility
        self.declare_parameter('invisible_threshold_percent', 0.8)  # Percentage of detections without target to be considered invisible

        # Bbox estimator (constant-velocity Kalman on center + size)
        self.declare_parameter('filter_process_std', 1.0)       # center accel noise (norm units/s^2)
        self.declare_parameter('filter_meas_std', 0.02)         # center detection noise (norm units)
        self.declare_parameter('filter_size_process_std', 0.5)  # size rate noise (norm units/s^2)
        self.declare_parameter('filter_size_meas_std', 0.02)    # size detection noise (norm units)
        self.declare_parameter('gate_threshold', 0.2)           # normalized dist to reject outliers
        self.declare_parameter('max_reject_count', 5)           # consecutive rejects before hard reset
        self.declare_parameter('use_kalman_filter', True)       # False -> raw detections, no filtering/coasting

        # Track-ID selection (tracker.detections IDs, see target_selector.py)
        self.declare_parameter('max_reacquire_distance', 0.08)  # normalized dist to adopt a churned ID
        self.declare_parameter('target_class_ids', [], ParameterDescriptor(dynamic_typing=True))  # empty -> accept all classes

        self.is_target_visible = False
        self.is_enabled = False

        # Detection history for visibility tracking
        self.detection_history = []  # List of (timestamp, is_detected) tuples

        # Bbox estimator (Kalman filter); gains synced from params each cycle
        self.estimator = BBoxEstimator(
            process_std=self.filter_process_std,
            meas_std=self.filter_meas_std,
            size_process_std=self.filter_size_process_std,
            size_meas_std=self.filter_size_meas_std,
            gate_threshold=self.gate_threshold,
            max_reject_count=self.max_reject_count,
        )

        # Track-ID selector (which ByteTrack ID to follow; see target_selector.py)
        self.selector = TargetSelector(
            candidate_window_sec=self.visible_window_sec,
            max_reacquire_distance=self.max_reacquire_distance,
        )

        # OSD state
        self.image_width = None
        self.image_height = None

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.detection_callback,
            qos.HIGH_FREQ_QOS
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos.HIGH_FREQ_QOS
        )
        self.yolo_rgb_enable = self.create_client(SetBool, self.yolo_enable_service)
        self.tracker_enable = self.create_client(SetBool, self.tracker_enable_service)

        self.target_visible_pub = self.create_publisher(
            Bool,
            '~/visible',
            qos.STATE_QOS
        )
        self.bbox_pub = self.create_publisher(
            Bbox2D,
            '~/bbox',
            qos.HIGH_FREQ_QOS
        )
        self.osd_pub = self.create_publisher(
            ImageAnnotations,
            '~/markers',
            qos.HIGH_FREQ_QOS
        )
        self.enable_srv = self.create_service(
            SetBool,
            '~/enable',
            self.enable_callback
        )
        
        self.add_on_set_parameters_callback(self.parameter_callback)

    # Parameter properties
    @property
    def detections_topic(self):
        return self.get_parameter('detections_topic').get_parameter_value().string_value

    @property
    def image_topic(self):
        return self.get_parameter('image_topic').get_parameter_value().string_value

    @property
    def yolo_enable_service(self):
        return self.get_parameter('yolo_enable_service').get_parameter_value().string_value

    @property
    def tracker_enable_service(self):
        return self.get_parameter('tracker_enable_service').get_parameter_value().string_value

    @property
    def visible_window_sec(self):
        return self.get_parameter('visible_window_sec').get_parameter_value().double_value
    
    @property
    def visible_threshold_percent(self):
        return self.get_parameter('visible_threshold_percent').get_parameter_value().double_value
    
    @property
    def invisible_window_sec(self):
        return self.get_parameter('invisible_window_sec').get_parameter_value().double_value
    
    @property
    def invisible_threshold_percent(self):
        return self.get_parameter('invisible_threshold_percent').get_parameter_value().double_value

    @property
    def filter_process_std(self):
        return self.get_parameter('filter_process_std').get_parameter_value().double_value

    @property
    def filter_meas_std(self):
        return self.get_parameter('filter_meas_std').get_parameter_value().double_value

    @property
    def filter_size_process_std(self):
        return self.get_parameter('filter_size_process_std').get_parameter_value().double_value

    @property
    def filter_size_meas_std(self):
        return self.get_parameter('filter_size_meas_std').get_parameter_value().double_value

    @property
    def gate_threshold(self):
        return self.get_parameter('gate_threshold').get_parameter_value().double_value

    @property
    def max_reject_count(self):
        return self.get_parameter('max_reject_count').get_parameter_value().integer_value

    @property
    def use_kalman_filter(self):
        return self.get_parameter('use_kalman_filter').get_parameter_value().bool_value

    @property
    def max_reacquire_distance(self):
        return self.get_parameter('max_reacquire_distance').get_parameter_value().double_value

    @property
    def target_class_ids(self) -> list:
        return [c for c in self.get_parameter('target_class_ids').value if isinstance(c, str) and c]

    RUNTIME_TUNABLE = {
        'filter_process_std', 'filter_meas_std',
        'filter_size_process_std', 'filter_size_meas_std',
        'gate_threshold', 'max_reject_count', 'use_kalman_filter',
        'max_reacquire_distance', 'target_class_ids',
    }
    UNIT_RANGE = {  # must be within [0.0, 1.0]
        'visible_threshold_percent', 'invisible_threshold_percent',
    }
    POSITIVE = {  # must be > 0.0
        'visible_window_sec', 'invisible_window_sec',
        'filter_meas_std', 'filter_size_meas_std', 'gate_threshold',
        'max_reacquire_distance',
    }
    NON_NEGATIVE = {  # must be >= 0.0
        'filter_process_std', 'filter_size_process_std', 'max_reject_count',
    }

    def parameter_callback(self, params):
        """Validate parameter changes, then apply their side effects atomically."""
        for param in params:
            if self.is_enabled and param.name not in self.RUNTIME_TUNABLE:
                return SetParametersResult(
                    successful=False,
                    reason=f"Cannot change {param.name} while tracker is enabled")
            if param.name in self.UNIT_RANGE and not (0.0 <= param.value <= 1.0):
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} must be between 0.0 and 1.0, got {param.value}")
            if param.name in self.POSITIVE and param.value <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} must be positive, got {param.value}")
            if param.name in self.NON_NEGATIVE and param.value < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{param.name} must be non-negative, got {param.value}")

        return SetParametersResult(successful=True)

    def enable_callback(self, request, response):
        """
        Service callback to enable or disable the target tracker.
        """
        if self.is_enabled == request.data:
            response.success = True
            response.message = "Target tracker already in requested state"
            return response

        self.is_enabled = request.data

        if request.data:
            response.success = True
            response.message = "Target tracker enabled"

            self.set_pipeline_enabled(True)
            self.publish_osd()
        else:
            response.success = True
            response.message = "Target tracker disabled"

            self.set_pipeline_enabled(False)

            # Clear detection history and estimator
            self.detection_history.clear()
            self.estimator.reset()
            self.selector.reset()

            # Reset state when disabling
            if self.is_target_visible:
                self.is_target_visible = False
                self.publish_target_visibility()

            self.publish_osd()

        return response

    def set_pipeline_enabled(self, enable: bool):
        """Enable/disable the upstream raw detector and the ID tracker together."""
        self.call_enable_service(self.yolo_rgb_enable, enable, "yolo")
        self.call_enable_service(self.tracker_enable, enable, "tracker")

    def call_enable_service(self, client, enable: bool, label: str):
        if not client.service_is_ready():
            self.get_logger().warn(f"{label} enable service not available")
            return

        request = SetBool.Request()
        request.data = enable
        client.call_async(request)

    def detection_callback(self, msg: Detection2DArray):
        # Ignore if not enabled
        if not self.is_enabled:
            return

        now = self.get_clock().now().nanoseconds / 1e9  # seconds

        # Peek before this cycle's update()/coast() call below mutates the filter.
        reference_center = self.estimator.peek_center(now)
        candidates = self.build_candidates(msg)

        self.selector.configure(self.visible_window_sec, self.max_reacquire_distance)
        prev_id = self.selector.current_id
        chosen = self.selector.select(candidates, now, reference_center)
        if prev_id is not None and self.selector.current_id != prev_id:
            self.get_logger().info(
                f"Target track id changed: {prev_id} -> {self.selector.current_id}")

        is_target_detected = chosen is not None

        self.process_detection(is_target_detected, chosen, now)

    def build_candidates(self, msg: Detection2DArray) -> list:
        """Convert tracker detections (pixel coords) into normalized Candidates.

        Detections whose class is not in `target_class_ids` are dropped before
        the selector ever sees them; an empty list accepts every class.
        """
        if not self.image_width or not self.image_height:
            return []

        allowed_classes = self.target_class_ids

        candidates = []
        for detection in msg.detections:
            if not detection.results:
                continue
            class_id = detection.results[0].hypothesis.class_id
            if allowed_classes and class_id not in allowed_classes:
                continue
            vbbox = detection.bbox
            candidates.append(Candidate(
                id=detection.id,
                score=detection.results[0].hypothesis.score,
                center_x=vbbox.center.position.x / self.image_width,
                center_y=vbbox.center.position.y / self.image_height,
                size_x=vbbox.size_x / self.image_width,
                size_y=vbbox.size_y / self.image_height,
            ))
        return candidates

    def process_detection(self, is_target_detected: bool, chosen: Optional[Candidate], current_time: float):
        """
        Update visibility state and publish the (filtered) target bbox.

        is_target_detected reflects whether the selector matched a detection this
        frame (drives the visibility window). The Kalman filter only runs while
        `is_target_visible` is True -- otherwise a stray detection during the
        search phase (before the target is confirmed present) could seed the
        estimator and have it coast indefinitely with no visible transition ever
        occurring to reset it.
        """
        # Add to detection history
        self.detection_history.append((current_time, is_target_detected))

        # Update visibility status based on detection history
        self.update_visibility_status(current_time)

        estimate = self.estimate_bbox(chosen, current_time) if self.is_target_visible else None

        # Update OSD with the filtered estimate when available
        if estimate is not None:
            self.publish_osd(estimate)
        else:
            self.publish_osd()

        if self.is_target_visible:
            self.bbox_pub.publish(estimate if estimate is not None else self.make_nan_bbox())

    def estimate_bbox(self, chosen: Optional[Candidate], now: float) -> Optional[Bbox2D]:
        """Run the bbox estimator on this frame's chosen detection (visible-only)."""
        measurement = None
        if chosen is not None:
            measurement = (chosen.center_x, chosen.center_y, chosen.size_x, chosen.size_y)

        if self.use_kalman_filter:
            # Keep the filter gains in sync with (possibly live-tuned) parameters.
            self.estimator.configure(
                self.filter_process_std, self.filter_meas_std,
                self.filter_size_process_std, self.filter_size_meas_std,
                self.gate_threshold, self.max_reject_count,
            )

            # Feed the estimator: correct on a real detection, otherwise coast the
            # prediction forward so short dropouts don't freeze the control loop.
            if measurement is not None:
                est = self.estimator.update(*measurement, now)
            else:
                est = self.estimator.coast(now)

            return self.bbox_from_estimate(est) if est is not None else None

        # Filter bypassed: publish raw detections and drop out immediately on a
        # missed frame (no smoothing, no coasting). Reset so the filter re-seeds
        # cleanly if it is switched back on mid-track.
        self.estimator.reset()
        return self.bbox_from_measurement(measurement) if measurement is not None else None

    # ── Bbox estimate helpers ──────────────────────────────────────────

    def make_nan_bbox(self) -> Bbox2D:
        bbox = Bbox2D()
        bbox.center_x = float('nan')
        bbox.center_y = float('nan')
        bbox.size_x = float('nan')
        bbox.size_y = float('nan')
        return bbox

    def bbox_from_estimate(self, est) -> Bbox2D:
        """Build a Bbox2D message from a BBoxEstimate."""
        bbox = Bbox2D()
        bbox.center_x = est.center_x
        bbox.center_y = est.center_y
        bbox.size_x = est.size_x
        bbox.size_y = est.size_y
        return bbox

    def bbox_from_measurement(self, measurement) -> Bbox2D:
        """Build a Bbox2D straight from a raw (unfiltered) detection tuple."""
        cx, cy, sx, sy = measurement
        bbox = Bbox2D()
        bbox.center_x = cx
        bbox.center_y = cy
        bbox.size_x = sx
        bbox.size_y = sy
        return bbox

    def update_visibility_status(self, current_time):
        """
        Update target visibility status based on detection history.
        Uses a sliding window approach with percentage thresholds.
        """
        # Clean old entries from detection history based on the larger window
        max_window = max(self.visible_window_sec, self.invisible_window_sec)
        self.detection_history = [
            (t, detected) for t, detected in self.detection_history
            if current_time - t <= max_window + 0.1  # small buffer
        ]

        if not self.detection_history:
            return
        
        # Calculate the collected detection to see if we have anough data.
        oldest_time = self.detection_history[0][0]
        collected_time = current_time - oldest_time

        if not self.is_target_visible and collected_time >= self.visible_window_sec:
            # Check if target should become visible
            window_start = current_time - self.visible_window_sec
            detections_in_window = [
                detected for t, detected in self.detection_history
                if t >= window_start
            ]
            
            if detections_in_window:
                detection_rate = sum(detections_in_window) / len(detections_in_window)
                if detection_rate >= self.visible_threshold_percent:
                    self.is_target_visible = True
                    self.publish_target_visibility()
                    self.detection_history.clear()

                    locked_id = self.selector.lock_best_candidate()
                    if locked_id:
                        self.get_logger().info(f"Target acquired, tracking id {locked_id}")
                    else:
                        self.get_logger().info("Target acquired (no track id available)")
        elif self.is_target_visible and collected_time >= self.invisible_window_sec:
            # Check if target should become invisible
            window_start = current_time - self.invisible_window_sec
            detections_in_window = [
                detected for t, detected in self.detection_history
                if t >= window_start
            ]
            
            if detections_in_window:
                non_detection_rate = 1.0 - (sum(detections_in_window) / len(detections_in_window))
                if non_detection_rate >= self.invisible_threshold_percent:
                    self.is_target_visible = False
                    self.publish_target_visibility()
                    self.detection_history.clear()
                    self.get_logger().info("Target lost, resuming search")

                    # Clear the estimator and track-ID selection so re-acquisition starts fresh
                    self.estimator.reset()
                    self.selector.reset()

    def publish_target_visibility(self):
        """
        Publish current target visibility status.
        """
        visibility_msg = Bool()
        visibility_msg.data = self.is_target_visible
        self.target_visible_pub.publish(visibility_msg)

    # ── OSD ────────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        """Receive camera resolution once, then unsubscribe."""
        self.image_width = msg.width
        self.image_height = msg.height
        self.publish_osd()

        # Only need the first message
        self.destroy_subscription(self.image_sub)
        self.image_sub = None

    def publish_osd(self, bbox: Bbox2D = None):
        """Build and publish OSD annotations; empty when disabled so the OSD clears."""
        if self.image_width is None or self.image_height is None:
            return

        annotations = ImageAnnotations()

        if self.is_enabled and bbox is not None:
            annotations.points.extend(self.make_target_marker(bbox))

        self.osd_pub.publish(annotations)

    def make_line_annotation(self, line_type, points, rgb):
        """Build a PointsAnnotation of the given type from pixel-space corners."""
        pts = PointsAnnotation()
        pts.type = line_type
        pts.thickness = 2.0
        pts.outline_color.r, pts.outline_color.g, pts.outline_color.b = rgb
        pts.outline_color.a = 1.0
        for x, y in points:
            p = Point2()
            p.x = float(x)
            p.y = float(y)
            pts.points.append(p)
        return pts

    def make_target_marker(self, bbox: Bbox2D):
        """Draw the filtered target bbox plus a corner-to-corner crosshair.

        Returns a LINE_LOOP for the box outline and a LINE_LIST whose point pairs
        form the two diagonals (an X centered on the target).
        """
        center_x = bbox.center_x * self.image_width
        center_y = bbox.center_y * self.image_height
        half_w = 0.5 * bbox.size_x * self.image_width
        half_h = 0.5 * bbox.size_y * self.image_height

        top_left = (center_x - half_w, center_y - half_h)
        top_right = (center_x + half_w, center_y - half_h)
        bottom_right = (center_x + half_w, center_y + half_h)
        bottom_left = (center_x - half_w, center_y + half_h)

        rgb = (0.0, 1.0, 0.0)
        box = self.make_line_annotation(
            PointsAnnotation.LINE_LOOP,
            [top_left, top_right, bottom_right, bottom_left], rgb)
        # LINE_LIST draws a segment per point pair: TL-BR and TR-BL -> an X.
        crosshair = self.make_line_annotation(
            PointsAnnotation.LINE_LIST,
            [top_left, bottom_right, top_right, bottom_left], rgb)
        return [box, crosshair]

def main(args=None):
    """Main entry point for the target tracker node."""
    rclpy.init(args=args)
    
    try:
        node = TargetTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in target tracker node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
