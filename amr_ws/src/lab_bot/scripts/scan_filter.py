#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserScanFilter180(Node):
    def __init__(self):
        super().__init__('laser_scan_filter_180')

        # ==========================================================
        # CHỌN 1 TRONG 4 HƯỚNG BẠN MUỐN LẤY 180 ĐỘ:
        # 'FRONT' : Nửa trên (chính diện mũi xe)
        # 'REAR'  : Nửa dưới (phía sau đuôi xe)
        # 'LEFT'  : Nửa bên trái
        # 'RIGHT' : Nửa bên phải
        # ==========================================================
        self.DIRECTION = 'REAR'

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.get_logger().info(f'Laser Scan Filter 180 đang chọn hướng: {self.DIRECTION}')

    def scan_callback(self, msg: LaserScan):
        total_pts = len(msg.ranges)
        if total_pts == 0:
            return

        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.angle_increment = msg.angle_increment

        quarter = total_pts // 4
        half = total_pts // 2

        # Cắt đúng 180 độ theo hướng đã chọn
        if self.DIRECTION == 'FRONT':
            # Nửa trên: từ -90° đến +90° (nửa mảng ở giữa)
            start_idx = quarter
            end_idx = quarter + half
            filtered_msg.ranges = list(msg.ranges[start_idx:end_idx])
            filtered_msg.intensities = list(msg.intensities[start_idx:end_idx]) if msg.intensities else []
            filtered_msg.angle_min = -math.pi / 2.0
            filtered_msg.angle_max = math.pi / 2.0

        elif self.DIRECTION == 'REAR':
            # Nửa dưới: từ +90° đến +270° (ghép 2 đầu mảng)
            filtered_msg.ranges = list(msg.ranges[quarter + half:]) + list(msg.ranges[:quarter])
            if msg.intensities:
                filtered_msg.intensities = list(msg.intensities[quarter + half:]) + list(msg.intensities[:quarter])
            else:
                filtered_msg.intensities = []
            filtered_msg.angle_min = math.pi / 2.0
            filtered_msg.angle_max = 3.0 * math.pi / 2.0

        elif self.DIRECTION == 'LEFT':
            # Nửa trái: từ 0° đến +180°
            start_idx = half
            end_idx = total_pts
            filtered_msg.ranges = list(msg.ranges[start_idx:end_idx])
            filtered_msg.intensities = list(msg.intensities[start_idx:end_idx]) if msg.intensities else []
            filtered_msg.angle_min = 0.0
            filtered_msg.angle_max = math.pi

        elif self.DIRECTION == 'RIGHT':
            # Nửa phải: từ -180° đến 0°
            start_idx = 0
            end_idx = half
            filtered_msg.ranges = list(msg.ranges[start_idx:end_idx])
            filtered_msg.intensities = list(msg.intensities[start_idx:end_idx]) if msg.intensities else []
            filtered_msg.angle_min = -math.pi
            filtered_msg.angle_max = 0.0

        else:
            self.get_logger().error(f'Hướng không hợp lệ: {self.DIRECTION}')
            return

        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilter180()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()