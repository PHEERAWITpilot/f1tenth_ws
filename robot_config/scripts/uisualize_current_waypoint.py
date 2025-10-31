#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped  # ✅ AMCL uses this type!
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import sys
import math

class CurrentWaypointVisualizer(Node):
    def __init__(self, waypoint_file):
        super().__init__('current_waypoint_visualizer')
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/current_waypoint_marker', 10)
        
        # ✅ Subscribe to AMCL pose - note the message type!
        self.robot_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,  # AMCL publishes this type
            '/amcl_pose',
            self.robot_pose_callback,
            10
        )
        
        # Load waypoints
        self.waypoint_file = waypoint_file
        self.waypoints = self.load_waypoints()
        self.current_waypoint_index = 0
        self.robot_pose = None
        self.completed_waypoints = set()
        
        # Timer for continuous publishing
        self.create_timer(0.1, self.update_current_waypoint)  # 10Hz
        
        self.get_logger().info(f'🎯 Tracking waypoint progress from: {waypoint_file}')
        self.get_logger().info(f'📊 Loaded {len(self.waypoints)} waypoints')
        self.get_logger().info('🤖 Listening to /amcl_pose for robot position')
        
    def load_waypoints(self):
        """Load waypoints from YAML file"""
        try:
            with open(self.waypoint_file, 'r') as f:
                data = yaml.safe_load(f)
            return data['waypoints']
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []
    
    def robot_pose_callback(self, msg):
        """Update robot's current position - AMCL message type"""
        self.robot_pose = msg.pose.pose  # AMCL wraps pose in PoseWithCovariance
    
    def distance_to_waypoint(self, waypoint_idx):
        """Calculate distance from robot to waypoint"""
        if self.robot_pose is None or waypoint_idx >= len(self.waypoints):
            return float('inf')
        
        wp = self.waypoints[waypoint_idx]
        dx = wp['position']['x'] - self.robot_pose.position.x
        dy = wp['position']['y'] - self.robot_pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def update_current_waypoint(self):
        """Update which waypoint is current based on robot distance"""
        if self.robot_pose is None or not self.waypoints:
            return
        
        # Check if robot has reached current waypoint
        # Using 0.5m to match your RemovePassedGoals radius
        current_dist = self.distance_to_waypoint(self.current_waypoint_index)
        
        if current_dist < 0.5:
            # Mark as completed
            self.completed_waypoints.add(self.current_waypoint_index)
            
            # Move to next waypoint if available
            if self.current_waypoint_index < len(self.waypoints) - 1:
                old_idx = self.current_waypoint_index
                self.current_waypoint_index += 1
                self.get_logger().info(
                    f'✅ Passed waypoint #{old_idx + 1}! '
                    f'Now targeting #{self.current_waypoint_index + 1}/{len(self.waypoints)}'
                )
            else:
                self.get_logger().info('🏁 Final waypoint reached!')
        
        # Publish marker
        self.publish_markers()
    
    def publish_markers(self):
        """Publish visual markers for current waypoint and status"""
        if not self.waypoints:
            return
        
        markers = MarkerArray()
        
        # Show all waypoints with status colors
        for i, wp in enumerate(self.waypoints):
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'waypoint_status'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            
            sphere.pose.position.x = wp['position']['x']
            sphere.pose.position.y = wp['position']['y']
            sphere.pose.position.z = 0.15
            
            # Color based on status
            if i in self.completed_waypoints:
                # ✅ Completed - Blue, smaller
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
                sphere.color.r = 0.2
                sphere.color.g = 0.5
                sphere.color.b = 1.0
                sphere.color.a = 0.7
            elif i == self.current_waypoint_index:
                # 🎯 Current - Pulsing Green, large
                pulse = 0.35 + 0.15 * math.sin(self.get_clock().now().nanoseconds / 1e9 * 4.0)
                sphere.scale.x = sphere.scale.y = sphere.scale.z = pulse
                sphere.color.r = 0.0
                sphere.color.g = 1.0
                sphere.color.b = 0.0
                sphere.color.a = 1.0
            else:
                # ⏳ Pending - Gray, small
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.12
                sphere.color.r = 0.6
                sphere.color.g = 0.6
                sphere.color.b = 0.6
                sphere.color.a = 0.5
            
            markers.markers.append(sphere)
        
        # Special marker for CURRENT waypoint - arrow + text
        if self.current_waypoint_index < len(self.waypoints):
            wp = self.waypoints[self.current_waypoint_index]
            
            # Yellow arrow showing target orientation
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'current_waypoint_arrow'
            arrow.id = 1000
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            
            arrow.pose.position.x = wp['position']['x']
            arrow.pose.position.y = wp['position']['y']
            arrow.pose.position.z = 0.5
            arrow.pose.orientation.x = wp['orientation']['x']
            arrow.pose.orientation.y = wp['orientation']['y']
            arrow.pose.orientation.z = wp['orientation']['z']
            arrow.pose.orientation.w = wp['orientation']['w']
            
            arrow.scale.x = 0.6  # Length
            arrow.scale.y = 0.12  # Width
            arrow.scale.z = 0.12  # Height
            
            arrow.color.r = 1.0
            arrow.color.g = 1.0
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            
            markers.markers.append(arrow)
            
            # Text label with distance
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'current_waypoint_label'
            text.id = 2000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            
            text.pose.position.x = wp['position']['x']
            text.pose.position.y = wp['position']['y']
            text.pose.position.z = 0.8
            
            text.scale.z = 0.25
            text.color.r = 0.0
            text.color.g = 0.0
            text.color.b = 1.0
            text.color.a = 1.0
            
            dist = self.distance_to_waypoint(self.current_waypoint_index)
            text.text = f"TARGET: #{self.current_waypoint_index + 1}/{len(self.waypoints)}\n{dist:.2f}m"
            
            markers.markers.append(text)
        
        self.marker_pub.publish(markers)

def main():
    if len(sys.argv) < 2:
        print("❌ Usage: python3 visualize_current_waypoint.py <waypoint_file.yaml>")
        print("Example: python3 visualize_current_waypoint.py ~/f1tenth_ws/recorded_paths/waypoints_20251029_212934.yaml")
        return
    
    rclpy.init()
    node = CurrentWaypointVisualizer(sys.argv[1])
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

