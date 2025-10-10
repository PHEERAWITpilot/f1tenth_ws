import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.publish_pose()

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position from RViz: Position(-0.949417, 0.22481, 0)
        msg.pose.pose.position.x = -0.949417
        msg.pose.pose.position.y = 0.22481
        msg.pose.pose.position.z = 0.0

        # Quaternion from RViz: Orientation(0, 0, -0.998554, 0.0537616)
        # Note: RViz shows (x, y, z, w) format
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = -0.998554
        msg.pose.pose.orientation.w = 0.0537616

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
            f'Published initial pose: x={-0.949417}, y={0.22481}, '
            f'orientation(z={-0.998554}, w={0.0537616}), yaw=-3.03402 rad'
        )

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin_once(node, timeout_sec=2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

