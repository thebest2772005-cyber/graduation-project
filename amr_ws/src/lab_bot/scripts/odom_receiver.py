#!/usr/bin/env python3

import math
import struct
import serial

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


def normalize_angle(angle):
    """Giới hạn góc về khoảng [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class UartOdomReceiver(Node):

    def __init__(self):
        super().__init__("odom_receiver")

        ###############################
        ## PARAMETERS
        ###############################

        self.declare_parameter("port", "/dev/mcu")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("position_scale", 1.0)

        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baudrate").value

        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame = self.get_parameter("child_frame_id").value
        self.position_scale = self.get_parameter("position_scale").value

        if self.position_scale <= 0.0:
            raise ValueError("position_scale must be greater than zero")

        ###############################
        ## Publisher
        ###############################

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            20
        )

        ###############################
        ## TF
        ###############################

        self.tf_broadcaster = TransformBroadcaster(self)

        ###############################
        ## Variables
        ###############################

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0

        self.last_time = self.get_clock().now()

        self.serial = None

        self.connect_serial()

        ###############################

        self.timer = self.create_timer(
            0.02,
            self.read_serial
        )

    ####################################################

    def connect_serial(self):
        while rclpy.ok():
            try:
                self.serial = serial.Serial(
                    self.port,
                    self.baud,
                    timeout=0.05
                )

                self.get_logger().info(
                    f"Connected : {self.port}"
                )

                break

            except Exception as e:
                self.get_logger().error(
                    f"Serial Error : {e}"
                )

                self.get_logger().info(
                    "Retry after 2 sec..."
                )

                import time
                time.sleep(2)

    ####################################################

    def publish_tf(self, x, y, theta):
        q = quaternion_from_euler(0, 0, theta)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    ####################################################

    def publish_odom(self, x, y, theta):
        now = self.get_clock().now()

        dt = (now - self.last_time).nanoseconds / 1e9

        if dt <= 0:
            dt = 0.02

        # 1. Tính khoảng dịch chuyển trong khung Odom
        dx = x - self.last_x
        dy = y - self.last_y

        # 2. Chiếu vận tốc về khung robot (base_link)
        vx_robot = (dx * math.cos(theta) + dy * math.sin(theta)) / dt
        vy_robot = (-dx * math.sin(theta) + dy * math.cos(theta)) / dt

        # Giới hạn theta về [-pi, pi]
        theta = normalize_angle(theta)

        # Tính vtheta
        dtheta = normalize_angle(theta - self.last_theta)
        vtheta = dtheta / dt

        q = quaternion_from_euler(0, 0, theta)

        msg = Odometry()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        msg.twist.twist.linear.x = vx_robot
        msg.twist.twist.linear.y = vy_robot
        msg.twist.twist.angular.z = vtheta

        msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03
        ]

        self.odom_pub.publish(msg)

        self.publish_tf(x, y, theta)

        self.last_x = x
        self.last_y = y
        self.last_theta = theta

        self.last_time = now

    ####################################################

    def read_serial(self):
        if self.serial is None:
            return

        max_frames_per_call = 5
        frames_processed = 0
        last_x = last_y = last_theta = 0.0

        try:
            while self.serial.in_waiting >= 14 and frames_processed < max_frames_per_call:
                head = self.serial.read(1)
                if head != b'\xAA':
                    continue

                frame = self.serial.read(13)
                if len(frame) != 13:
                    return

                if frame[12] != 0x0D:
                    self.serial.reset_input_buffer()
                    break

                payload = frame[0:12]
                raw_x, raw_y, raw_theta = struct.unpack("<fff", payload)

                x = raw_x * self.position_scale
                y = raw_y * self.position_scale
                theta = normalize_angle(raw_theta)

                self.publish_odom(x, y, theta)

                last_x, last_y, last_theta = x, y, theta
                frames_processed += 1

            # Log NGOÀI vòng while — chỉ 1 lần mỗi lần callback chạy, có throttle
            if frames_processed > 0:
                self.get_logger().info(
                    f"odom: {frames_processed} frames | x={last_x:.3f} y={last_y:.3f} theta={last_theta:.3f}",
                    throttle_duration_sec=1.0
                )

        except serial.SerialException:
            self.get_logger().error("Lost Serial...")
            self.connect_serial()
        except Exception as e:
            self.get_logger().warn(str(e), throttle_duration_sec=1.0)

########################################################


def main(args=None):
    rclpy.init(args=args)

    node = UartOdomReceiver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node.serial is not None:
            node.serial.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()