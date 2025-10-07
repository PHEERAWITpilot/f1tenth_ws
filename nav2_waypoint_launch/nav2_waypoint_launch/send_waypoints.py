import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import math

def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))

WAYPOINTS = [
    (-2.13, -2.05, 0.0054),
    (-0.419, -0.63, 0.0482),
    (1.8, -0.159, 0.108),
    (3.02, -0.265, 0.00477),
    (2.53, -3.01, 0.00202),
    (-0.766, -3.49, 0.206),
]

class WaypointClient(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self.cli = ActionClient(self, FollowWaypoints, '/waypoint_follower')
        self.cli.wait_for_server()
        poses = []
        for x, y, yaw in WAYPOINTS:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            q = yaw_to_quat(yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            poses.append(pose)
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
        self.send_goal_future = self.cli.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

