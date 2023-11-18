#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

from motion_capture_tracking_interfaces.msg import NamedPoseArray, NamedPose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        # パブリッシャーを作成
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            deadline=Duration(seconds=0, nanoseconds=1e9 / 100.0)
        )
        self.publisher = self.create_publisher(NamedPoseArray, '/poses', qos_profile)
        # タイマーを作成（1秒ごとにcallbackを呼び出す）
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        # NamedPoseArrayメッセージを作成
        named_pose_array_msg = NamedPoseArray()

        # Headerを設定 (seqは自動で設定されます)
        named_pose_array_msg.header = Header()
        named_pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        named_pose_array_msg.header.frame_id = "world"
        pose_cf231 = Pose()
        pose_cf231.position.x = 0.0
        pose_cf231.position.y = 0.0
        pose_cf231.position.z = 1.0
        pose_cf231.orientation.x = 0.0
        pose_cf231.orientation.y = 0.0
        pose_cf231.orientation.z = 0.0
        pose_cf231.orientation.w = 1.0

        # NamedPoseメッセージのリストを作成
        named_pose_array_msg.poses = [
            NamedPose(name="cf231", pose=pose_cf231),
        ]
        
        # メッセージをログに出力して確認
        self.get_logger().info(f'Publishing: {named_pose_array_msg}')
        
        # メッセージをパブリッシュ
        self.publisher.publish(named_pose_array_msg)

def main(args=None):
    rclpy.init(args=args)   # ROS 2 Pythonクライアントライブラリを初期化
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)  # ノードをspinさせ続けます

    # シャットダウン時のクリーンアップ
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()