#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class PreciseWaypointFollower(Node):
    def __init__(self):
        super().__init__('precise_waypoint_follower')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Your 31 waypoints (from your bash script)
        self.waypoints = [
            {'x': -2.457, 'y': 1.144, 'z': -0.015, 'w': 1.0},
            {'x': -1.94, 'y': 1.101, 'z': -0.019, 'w': 1.0},
            {'x': -1.395, 'y': 1.054, 'z': -0.022, 'w': 1.0},
            {'x': -0.884, 'y': 1.015, 'z': -0.021, 'w': 1.0},
            {'x': -0.389, 'y': 0.946, 'z': -0.068, 'w': 0.998},
            {'x': 0.124, 'y': 0.824, 'z': -0.136, 'w': 0.991},
            {'x': 0.627, 'y': 0.631, 'z': -0.197, 'w': 0.98},
            {'x': 1.037, 'y': 0.323, 'z': -0.388, 'w': 0.922},
            {'x': 1.287, 'y': -0.112, 'z': -0.555, 'w': 0.832},
            {'x': 1.399, 'y': -0.633, 'z': -0.671, 'w': 0.742},
            {'x': 1.361, 'y': -1.149, 'z': -0.75, 'w': 0.662},
            {'x': 1.188, 'y': -1.634, 'z': -0.831, 'w': 0.557},
            {'x': 0.859, 'y': -2.038, 'z': -0.941, 'w': 0.338},
            {'x': 0.423, 'y': -2.292, 'z': -0.993, 'w': 0.118},
            {'x': -0.074, 'y': -2.234, 'z': 0.994, 'w': 0.107},
            {'x': -0.58, 'y': -2.174, 'z': 0.997, 'w': 0.083},
            {'x': -1.092, 'y': -2.049, 'z': 0.996, 'w': 0.088},
            {'x': -1.596, 'y': -1.943, 'z': 0.996, 'w': 0.093},
            {'x': -2.117, 'y': -1.802, 'z': 0.992, 'w': 0.125},
            {'x': -2.67, 'y': -1.661, 'z': 0.993, 'w': 0.116},
            {'x': -3.206, 'y': -1.55, 'z': 0.995, 'w': 0.099},
            {'x': -3.684, 'y': -1.338, 'z': 0.962, 'w': 0.275},
            {'x': -4.015, 'y': -0.958, 'z': 0.868, 'w': 0.496},
            {'x': -4.121, 'y': -0.454, 'z': 0.726, 'w': 0.688},
            {'x': -4.029, 'y': 0.045, 'z': 0.571, 'w': 0.821},
            {'x': -3.777, 'y': 0.504, 'z': 0.405, 'w': 0.914},
            {'x': -3.328, 'y': 0.759, 'z': 0.17, 'w': 0.985},
            {'x': -2.821, 'y': 0.947, 'z': 0.2, 'w': 0.98},
            {'x': -2.346, 'y': 1.145, 'z': 0.202, 'w': 0.979},
            {'x': -1.839, 'y': 1.172, 'z': -0.07, 'w': 0.998},
            {'x': -1.336, 'y': 1.001, 'z': -0.196, 'w': 0.981}
        ]
        
        self.current_waypoint = 0
        self.loop_count = 0
        
    def send_goal(self, waypoint_index):
        """Send a single waypoint as a goal"""
        wp = self.waypoints[waypoint_index]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp['x']
        goal_msg.pose.pose.position.y = wp['y']
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = wp['z']
        goal_msg.pose.pose.orientation.w = wp['w']
        
        self.get_logger().info(f'Loop {self.loop_count + 1} - Waypoint {waypoint_index + 1}/31: ({wp["x"]:.2f}, {wp["y"]:.2f})')
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
            
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def feedback_callback(self, feedback_msg):
        pass  # Optional: print distance to goal
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'✅ Waypoint {self.current_waypoint + 1} reached!')
            
            # Move to next waypoint
            self.current_waypoint += 1
            
            # If finished all waypoints, restart loop
            if self.current_waypoint >= len(self.waypoints):
                self.loop_count += 1
                self.current_waypoint = 0
                self.get_logger().info(f'🏁 Loop {self.loop_count} completed! Starting loop {self.loop_count + 1}...')
                time.sleep(0.5)  # Brief pause between loops
            
            # Send next waypoint
            self.send_goal(self.current_waypoint)
        else:
            self.get_logger().error(f'❌ Waypoint {self.current_waypoint + 1} failed with status {status}')
            # Retry same waypoint
            time.sleep(1.0)
            self.send_goal(self.current_waypoint)
            
def main(args=None):
    rclpy.init(args=args)
    node = PreciseWaypointFollower()
    
    # Start with first waypoint
    node.send_goal(0)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

