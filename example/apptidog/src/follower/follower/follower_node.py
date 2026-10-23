#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
import shared_qos as qos

from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from dog_msgs.msg import Bbox2D

from simple_pid import PID


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Parameters
        self.declare_parameter('control_loop_hz', 20.0)
        self.declare_parameter('desired_x', 0.5)
        self.declare_parameter('angular_pid.kp', 3.0)
        self.declare_parameter('angular_pid.ki', 0.0)
        self.declare_parameter('angular_pid.kd', 0.1)
        self.declare_parameter('max_angular_z', 1.0)
        self.declare_parameter('desired_size_x', 0.2)
        self.declare_parameter('linear_pid.kp', 2.0)
        self.declare_parameter('linear_pid.ki', 0.0)
        self.declare_parameter('linear_pid.kd', 0.05)
        self.declare_parameter('max_linear_x', 1.0)

        self.is_enabled = False
        self.is_target_visible = False
        self.bbox: Bbox2D = None

        self.angular_pid = PID(
            Kp=self.angular_kp,
            Ki=self.angular_ki,
            Kd=self.angular_kd,
            setpoint=self.desired_x,
            output_limits=(-self.max_angular_z, self.max_angular_z),
        )

        self.linear_pid = PID(
            Kp=self.linear_kp,
            Ki=self.linear_ki,
            Kd=self.linear_kd,
            setpoint=math.log(self.desired_size_x),
            output_limits=(-self.max_linear_x, self.max_linear_x),
        )

        self.bbox_sub = self.create_subscription(
            Bbox2D,
            '/target_tracker/bbox',
            self.bbox_callback,
            qos.HIGH_FREQ_QOS,
        )
        self.visible_sub = self.create_subscription(
            Bool,
            '/target_tracker/visible',
            self.visible_callback,
            qos.STATE_QOS,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_out',
            qos.RELIABLE_QOS,
        )
        self.enabled_pub = self.create_publisher(
            Bool,
            '~/enabled',
            qos.STATE_QOS,
        )

        self.target_tracker_enable = self.create_client(
            SetBool, '/target_tracker/enable'
        )
        self.enable_srv = self.create_service(
            SetBool, '~/enable', self.enable_callback
        )

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.control_loop_timer = self.create_timer(
            1.0 / self.control_loop_hz, self.control_loop
        )

        self.publish_enabled()

        self.get_logger().info('Follower Node started')

    # ── Parameter properties ────────────────────────────────────────────

    @property
    def control_loop_hz(self):
        return self.get_parameter('control_loop_hz').get_parameter_value().double_value

    @property
    def desired_x(self):
        return self.get_parameter('desired_x').get_parameter_value().double_value

    @property
    def angular_kp(self):
        return self.get_parameter('angular_pid.kp').get_parameter_value().double_value

    @property
    def angular_ki(self):
        return self.get_parameter('angular_pid.ki').get_parameter_value().double_value

    @property
    def angular_kd(self):
        return self.get_parameter('angular_pid.kd').get_parameter_value().double_value

    @property
    def max_angular_z(self):
        return self.get_parameter('max_angular_z').get_parameter_value().double_value

    @property
    def desired_size_x(self):
        return self.get_parameter('desired_size_x').get_parameter_value().double_value

    @property
    def linear_kp(self):
        return self.get_parameter('linear_pid.kp').get_parameter_value().double_value

    @property
    def linear_ki(self):
        return self.get_parameter('linear_pid.ki').get_parameter_value().double_value

    @property
    def linear_kd(self):
        return self.get_parameter('linear_pid.kd').get_parameter_value().double_value

    @property
    def max_linear_x(self):
        return self.get_parameter('max_linear_x').get_parameter_value().double_value

    # ── Enable / disable ─────────────────────────────────────────────

    def enable_callback(self, request, response):
        if self.is_enabled == request.data:
            response.success = True
            response.message = "Follower already in requested state"
            return response

        self.is_enabled = request.data

        if request.data:
            self.get_logger().info("Enabling follower")
            self._call_enable_target_tracker(True)
        else:
            self.get_logger().info("Disabling follower")
            self._call_enable_target_tracker(False)
            self.is_target_visible = False
            self.bbox = None
            self.angular_pid.reset()
            self.linear_pid.reset()

        self.publish_enabled()

        response.success = True
        response.message = "Follower enabled" if request.data else "Follower disabled"
        return response

    def publish_enabled(self):
        msg = Bool()
        msg.data = self.is_enabled
        self.enabled_pub.publish(msg)

    def _call_enable_target_tracker(self, enable: bool):
        request = SetBool.Request()
        request.data = enable
        if self.target_tracker_enable.service_is_ready():
            self.target_tracker_enable.call_async(request)
        else:
            self.get_logger().warn("Target tracker enable service not available")

    # ── Callbacks ───────────────────────────────────────────────────────

    def visible_callback(self, msg: Bool):
        if not self.is_enabled:
            return
        self.is_target_visible = msg.data
        if not self.is_target_visible:
            self.bbox = None
            self.angular_pid.reset()
            self.linear_pid.reset()

    def bbox_callback(self, msg: Bbox2D):
        if not self.is_enabled:
            return
        if math.isnan(msg.center_x) or math.isnan(msg.center_y):
            return
        self.bbox = msg

    # ── Control loop ────────────────────────────────────────────────────

    def control_loop(self):
        if not self.is_enabled:
            return
        if not self.is_target_visible or self.bbox is None:
            return

        angular_z = self.angular_pid(self.bbox.center_x)

        twist = Twist()
        twist.angular.z = float(angular_z)

        if self.bbox.size_x > 0.0 and not math.isnan(self.bbox.size_x):
            linear_x = self.linear_pid(math.log(self.bbox.size_x))
            twist.linear.x = float(linear_x)

        self.cmd_vel_pub.publish(twist)

    # ── Parameter change callback ────────────────────────────────────────

    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult

        new_values = {p.name: p.value for p in params}

        def get(name):
            return new_values.get(name, self.get_parameter(name).get_parameter_value().double_value)

        for param in params:
            if param.name in ('angular_pid.kp', 'angular_pid.ki', 'angular_pid.kd'):
                self.angular_pid.tunings = (
                    get('angular_pid.kp'), get('angular_pid.ki'), get('angular_pid.kd')
                )
            elif param.name in ('linear_pid.kp', 'linear_pid.ki', 'linear_pid.kd'):
                self.linear_pid.tunings = (
                    get('linear_pid.kp'), get('linear_pid.ki'), get('linear_pid.kd')
                )
            elif param.name == 'desired_x':
                self.angular_pid.setpoint = get('desired_x')
            elif param.name == 'desired_size_x':
                v = get('desired_size_x')
                self.linear_pid.setpoint = math.log(v)
            elif param.name == 'max_angular_z':
                limit = get('max_angular_z')
                self.angular_pid.output_limits = (-limit, limit)
            elif param.name == 'max_linear_x':
                limit = get('max_linear_x')
                self.linear_pid.output_limits = (-limit, limit)
            elif param.name == 'control_loop_hz':
                self.control_loop_timer.cancel()
                self.control_loop_timer = self.create_timer(
                    1.0 / get('control_loop_hz'), self.control_loop
                )

        result = SetParametersResult()
        result.successful = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
