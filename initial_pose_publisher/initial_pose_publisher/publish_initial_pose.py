#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Wait for subscribers (AMCL) to be ready
        time.sleep(0.5)
        self.publish_pose()

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position from RViz: (0.801462, -1.53279)
        msg.pose.pose.position.x = 0.801462
        msg.pose.pose.position.y = -1.53279
        msg.pose.pose.position.z = 0.0

        # Orientation: yaw = -1.68311 rad
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = -0.745682
        msg.pose.pose.orientation.w = 0.666302

        # Covariance
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0685, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0685, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]
        
        self.publisher_.publish(msg)
        self.get_logger().info(
            '✅ Published initial pose: x=0.801462, y=-1.53279, yaw=-1.68311 rad'
        )

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    
    # Keep node alive to ensure message delivery
    time.sleep(1.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

