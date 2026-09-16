#!/usr/bin/env python3
"""
Đọc file waypoints.yaml và gửi tuần tự cho Nav2 qua FollowWaypoints action.

Cách chạy:
    python3 run_waypoints.py
    # hoặc chỉ định file khác:
    python3 run_waypoints.py /duong/dan/waypoints.yaml

Yêu cầu:
    - Nav2 stack (bringup) đã chạy và robot đã localize (AMCL có pose ổn định)
    - Package nav2_simple_commander đã cài (thường có sẵn trong nav2_bringup)
"""

import sys
import yaml
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


def load_waypoints(yaml_path):
    """Đọc file waypoints.yaml, trả về danh sách PoseStamped."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    poses = []
    for wp in data['waypoints']:
        p = wp['pose']
        pose = PoseStamped()
        pose.header.frame_id = p['header']['frame_id']
        pose.pose.position.x = float(p['pose']['position']['x'])
        pose.pose.position.y = float(p['pose']['position']['y'])
        pose.pose.position.z = float(p['pose']['position'].get('z', 0.0))
        pose.pose.orientation.x = float(p['pose']['orientation'].get('x', 0.0))
        pose.pose.orientation.y = float(p['pose']['orientation'].get('y', 0.0))
        pose.pose.orientation.z = float(p['pose']['orientation'].get('z', 0.0))
        pose.pose.orientation.w = float(p['pose']['orientation'].get('w', 1.0))
        poses.append(pose)
    return poses


def main():
    yaml_path = sys.argv[1] if len(sys.argv) > 1 else 'waypoints.yaml'

    rclpy.init()
    navigator = BasicNavigator()

    print(f"Đang đọc waypoint từ: {yaml_path}")
    waypoints = load_waypoints(yaml_path)
    print(f"Đã nạp {len(waypoints)} waypoint.")

    # Gán timestamp hiện tại cho mỗi pose ngay trước khi gửi
    now = navigator.get_clock().now().to_msg()
    for wp in waypoints:
        wp.header.stamp = now

    print("Đợi Nav2 sẵn sàng (active)...")
    navigator.waitUntilNav2Active()

    print("Bắt đầu gửi waypoint...")
    navigator.followWaypoints(waypoints)

    # Vòng lặp theo dõi tiến trình
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(
                f"Đang đi tới waypoint thứ {feedback.current_waypoint + 1}/{len(waypoints)}"
            )
        rclpy.spin_once(navigator, timeout_sec=0.5)

    result = navigator.getResult()
    print(f"Kết quả cuối: {result}")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()