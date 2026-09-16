#!/usr/bin/env python3

import struct
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelSender(Node):

    def __init__(self):
        super().__init__("cmd_vel_sender")

        port = "/dev/stm32_cmd"
        baud = 115200

        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
        except Exception as e:
            self.get_logger().error(str(e))
            raise

        # Lưu giữ trạng thái tốc độ hiện tại (Latch value)
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Subscribe topic /cmd_vel từ teleop
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10
        )

        # Timer 50Hz (20ms) liên tục gửi tốc độ hiện tại xuống STM32
        # Giúp STM32 luôn thấy tín hiệu "sống" và KHÔNG BAO GIỜ bị Timeout
        self.send_period = 0.02  
        self.timer = self.create_timer(self.send_period, self.send_uart_frame)

    def cmd_callback(self, msg: Twist):
        # Cập nhật giá trị
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z

    def send_uart_frame(self):
        # Đóng gói và gửi liên tục mỗi 20ms
        frame = bytearray()
        frame.append(0xAA)
        frame.extend(struct.pack("<f", self.current_linear))
        frame.extend(struct.pack("<f", self.current_angular))
        frame.append(0x0D)

        try:
            self.ser.write(frame)
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f"Error writing to serial: {e}")


def main(args=None):

    rclpy.init(args=args)
    node = CmdVelSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Khi nhấn Ctrl+C ngắt node, gửi lệnh dừng hẳn cho an toàn
        stop_frame = bytearray([0xAA, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D])
        try:
            node.ser.write(stop_frame)
            node.ser.flush()
            node.ser.close()
        except Exception:
            pass
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()