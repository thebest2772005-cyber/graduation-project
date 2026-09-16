#!/usr/bin/env python3

import math
import struct
import serial

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


class UartOdomReceiver(Node):

    def __init__(self):
        super().__init__("uart_odom_receiver")

        # MCU currently reports the travelled distance at one tenth of a metre
        # (e.g. 0.1 when the robot has travelled 1.0 m).  Keep this as a ROS
        # parameter so it can be calibrated without changing the protocol.
        self.declare_parameter("port", "/dev/mcu")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("position_scale", 10.0)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value
        self.position_scale = self.get_parameter("position_scale").value

        if self.position_scale <= 0.0:
            raise ValueError("position_scale must be greater than zero")

        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Opened {port}")
        except Exception as e:
            self.get_logger().error(f"Cannot open serial: {e}")
            raise

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.read_serial_data)

        self.get_logger().info("Waiting UART data...")

    def get_quaternion(self, yaw):

        q = Quaternion()

        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)

        return q

    def read_serial_data(self):

        while self.serial_port.in_waiting >= 14:

            header = self.serial_port.read(1)

            if header != b"\xAA":
                continue

            frame = self.serial_port.read(13)

            if len(frame) != 13:
                continue

            if frame[-1] != 0x0D:
                self.serial_port.reset_input_buffer()
                continue

            try:

                raw_x, raw_y, theta = struct.unpack("<fff", frame[:12])
                x = raw_x * self.position_scale
                y = raw_y * self.position_scale

                self.get_logger().info(
                    "RX -> "
                    f"raw_x={raw_x:.3f} raw_y={raw_y:.3f}; "
                    f"x={x:.3f} y={y:.3f} theta={theta:.3f}"
                )

                self.publish_odom(x, y, theta)

            except Exception as e:

                self.get_logger().error(str(e))

    def publish_odom(self, x, y, theta):

        now = self.get_clock().now().to_msg()

        quat = self.get_quaternion(theta)

        tf = TransformStamped()

        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0

        tf.transform.rotation = quat

        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()

        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = quat

        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

        self.get_logger().info(
            f"Published /odom : x={x:.3f} y={y:.3f} theta={theta:.3f}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = UartOdomReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.serial_port.close()
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
