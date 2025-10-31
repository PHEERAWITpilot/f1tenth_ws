#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import Costmap
import yaml
import time

class BlockedWaypointFilter(Node):
    def __init__(self):
        super().__init__('blocked_waypoint_filter')
        
        # Subscribe to costmap
        self.costmap_sub = self.create_subscription(
            Costmap,
            '/global_costmap/costmap_raw',
            self.costmap_callback,
            10
        )
        
        # 🔥 Use NavigateThroughPoses action (same as .sh!)
        self.nav_client = ActionClient(
            self, 
            NavigateThroughPoses, 
            'navigate_through_poses'
        )
        
        self.costmap_data = None
        self.get_logger().info('🔍 Waiting for costmap data...')
    
    def costmap_callback(self, msg):
        self.costmap_data = msg
        if not hasattr(self, '_costmap_received_logged'):
            self.get_logger().info('✅ Costmap received!')
            self._costmap_received_logged = True
    
    def world_to_costmap(self, x, y):
        if not self.costmap_data:
            return None, None
        
        origin_x = self.costmap_data.metadata.origin.position.x
        origin_y = self.costmap_data.metadata.origin.position.y
        resolution = self.costmap_data.metadata.resolution
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        return grid_x, grid_y
    
    def get_waypoint_cost(self, x, y):
        if not self.costmap_data:
            return 0.0
        
        grid_x, grid_y = self.world_to_costmap(x, y)
        
        if grid_x is None:
            return 0.0
        
        width = self.costmap_data.metadata.size_x
        height = self.costmap_data.metadata.size_y
        
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return 0.0
        
        index = grid_y * width + grid_x
        
        if index < len(self.costmap_data.data):
            return float(self.costmap_data.data[index])
        
        return 0.0
    
    def filter_and_publish_waypoints(self, waypoint_file):
        # Wait for costmap
        self.get_logger().info('⏳ Waiting for costmap data...')
        timeout = 10.0
        start_time = time.time()
        while self.costmap_data is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.costmap_data is None:
            self.get_logger().error('❌ Timeout waiting for costmap!')
            return
        
        # Load waypoints
        with open(waypoint_file, 'r') as f:
            data = yaml.safe_load(f)
        
        original_waypoints = data['waypoints']
        valid_waypoints = []
        
        self.get_logger().info(f'📋 Checking {len(original_waypoints)} waypoints...')
        self.get_logger().info('─' * 60)
        
        # Check each waypoint
        for i, wp in enumerate(original_waypoints):
            x = wp['position']['x']
            y = wp['position']['y']
            cost = self.get_waypoint_cost(x, y)
            
            if cost < 200.0:
                valid_waypoints.append(wp)
                self.get_logger().info(
                    f'  ✅ Waypoint {i+1:2d}: ({x:5.2f}, {y:5.2f}) - VALID (cost={cost:3.0f})'
                )
            else:
                self.get_logger().warn(
                    f'  🚫 Waypoint {i+1:2d}: ({x:5.2f}, {y:5.2f}) - BLOCKED (cost={cost:3.0f}) - SKIPPED!'
                )
        
        self.get_logger().info('─' * 60)
        self.get_logger().info(
            f'📊 Result: {len(valid_waypoints)}/{len(original_waypoints)} waypoints valid'
        )
        
        if len(valid_waypoints) == 0:
            self.get_logger().error('❌ No valid waypoints!')
            return
        
        # Convert to Nav2 goals
        goal_msg = NavigateThroughPoses.Goal()
        for wp in valid_waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp['position']['x']
            pose.pose.position.y = wp['position']['y']
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = wp['orientation']['x']
            pose.pose.orientation.y = wp['orientation']['y']
            pose.pose.orientation.z = wp['orientation']['z']
            pose.pose.orientation.w = wp['orientation']['w']
            goal_msg.poses.append(pose)
        
        # Wait for action server
        self.get_logger().info('🚀 Sending filtered waypoints to Nav2...')
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Navigate action server not available!')
            return
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected!')
            return
        
        self.get_logger().info('✅ Goal accepted! Robot navigating!')
        self.get_logger().info('🏎️  Watch your robot move through valid waypoints!')
        self.get_logger().info('   Press Ctrl+C to cancel navigation')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = feedback.number_of_poses_remaining
        distance = feedback.distance_remaining
        
        self.get_logger().info(
            f'📍 Progress: {remaining} waypoints remaining, {distance:.2f}m to go'
        )
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('🏁 Navigation complete!')

def main():
    import sys
    
    rclpy.init()
    node = BlockedWaypointFilter()
    
    # Get waypoint file
    if len(sys.argv) > 1:
        waypoint_file = sys.argv[1]
    else:
        waypoint_file = '/home/f2/f1tenth_ws/recorded_paths/waypoints_20251029_234556.yaml'
    
    # Wait for initialization
    time.sleep(1)
    
    # Filter and publish
    node.filter_and_publish_waypoints(waypoint_file)
    
    # Keep node alive
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

