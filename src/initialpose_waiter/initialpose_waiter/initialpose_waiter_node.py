import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPoseWaiter(Node):
    def __init__(self):
        super().__init__('initialpose_waiter')
        self.pose_received = False
        self.subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Initial pose received, proceeding with lifecycle manager start.')
        self.pose_received = True
        rclpy.shutdown()  # Exit node upon receiving initial pose

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseWaiter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

