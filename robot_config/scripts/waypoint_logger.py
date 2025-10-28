#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
from datetime import datetime
import os

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        
        # Subscribe to AMCL pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.waypoints = []
        self.last_pose = None
        self.min_distance = 0.5  # Record waypoint every 0.5 meters
        
        self.get_logger().info('🏁 Waypoint Logger Started!')
        self.get_logger().info(f'📍 Recording waypoints every {self.min_distance}m')
        self.get_logger().info('🎮 Drive your robot now! Press Ctrl+C when done.')
        
    def pose_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z_ori = msg.pose.pose.orientation.z
        current_w_ori = msg.pose.pose.orientation.w
        
        # Check if we should log this waypoint
        should_log = False
        
        if len(self.waypoints) == 0:
            # First waypoint
            should_log = True
        else:
            # Calculate distance from last waypoint
            last_wp = self.waypoints[-1]
            dx = current_x - last_wp['position']['x']
            dy = current_y - last_wp['position']['y']
            dist = (dx**2 + dy**2)**0.5
            
            if dist >= self.min_distance:
                should_log = True
        
        if should_log:
            wp = {
                'position': {
                    'x': round(current_x, 3),
                    'y': round(current_y, 3),
                    'z': 0.0
                },
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': round(current_z_ori, 3),
                    'w': round(current_w_ori, 3)
                }
            }
            
            self.waypoints.append(wp)
            self.get_logger().info(
                f'✅ Waypoint #{len(self.waypoints)}: '
                f'({wp["position"]["x"]:.2f}, {wp["position"]["y"]:.2f})'
            )
        
    def save_waypoints(self):
        if len(self.waypoints) == 0:
            self.get_logger().warn('⚠️  No waypoints recorded!')
            return
        
        # Create output directory
        output_dir = os.path.expanduser('~/f1tenth_ws/recorded_paths')
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Save YAML file
        yaml_file = f'{output_dir}/waypoints_{timestamp}.yaml'
        with open(yaml_file, 'w') as f:
            yaml.dump({'waypoints': self.waypoints}, f, default_flow_style=False)
        
        self.get_logger().info(f'💾 Saved {len(self.waypoints)} waypoints to:')
        self.get_logger().info(f'   {yaml_file}')
        
        # Generate ROS2 command
        self.generate_command(output_dir, timestamp)
        
    def generate_command(self, output_dir, timestamp):
        cmd = 'ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses "{poses: ['
        
        for wp in self.waypoints:
            cmd += '{header: {frame_id: \'map\'}, pose: {position: '
            cmd += f'{{x: {wp["position"]["x"]}, y: {wp["position"]["y"]}, z: 0.0}}, '
            cmd += f'orientation: {{x: 0.0, y: 0.0, '
            cmd += f'z: {wp["orientation"]["z"]}, w: {wp["orientation"]["w"]}}}}}}}, '
        
        cmd = cmd[:-2]  # Remove trailing comma and space
        cmd += ']}" --feedback'
        
        # Save command file
        cmd_file = f'{output_dir}/replay_{timestamp}.sh'
        with open(cmd_file, 'w') as f:
            f.write('#!/bin/bash\n')
            f.write(f'# Recorded path with {len(self.waypoints)} waypoints\n')
            f.write(f'# Timestamp: {timestamp}\n\n')
            f.write(cmd + '\n')
        
        os.chmod(cmd_file, 0o755)  # Make executable
        
        self.get_logger().info(f'🎬 Generated replay script:')
        self.get_logger().info(f'   {cmd_file}')
        self.get_logger().info('')
        self.get_logger().info('🏁 To replay this path, run:')
        self.get_logger().info(f'   bash {cmd_file}')

def main():
    rclpy.init()
    node = WaypointLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('')
        node.get_logger().info('🛑 Stopping recording...')
        node.save_waypoints()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
