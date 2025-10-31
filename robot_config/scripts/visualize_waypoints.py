#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import yaml
import sys

class WaypointVisualizer(Node):
    def __init__(self, waypoint_file):
        super().__init__('waypoint_visualizer')
        
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.waypoint_file = waypoint_file
        
        # Timer to republish waypoints (updates when file changes)
        self.create_timer(1.0, self.publish_waypoints)
        
        self.get_logger().info(f'🎨 Visualizing waypoints from: {waypoint_file}')
        self.get_logger().info('📍 Waypoints will appear in RViz on topic: /waypoint_markers')
        self.get_logger().info('🔄 File is reloaded every 1 second - edit and see updates live!')
        
    def publish_waypoints(self):
        try:
            with open(self.waypoint_file, 'r') as f:
                data = yaml.safe_load(f)
            
            waypoints = data['waypoints']
            markers = MarkerArray()
            
            # Create path line
            line_marker = Marker()
            line_marker.header.frame_id = 'map'
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = 'waypoint_path'
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05  # Line width
            line_marker.color.r = 0.0
            line_marker.color.g = 0.8
            line_marker.color.b = 1.0
            line_marker.color.a = 1.0
            
            for wp in waypoints:
                p = Point()
                p.x = wp['position']['x']
                p.y = wp['position']['y']
                p.z = 0.1
                line_marker.points.append(p)
            
            markers.markers.append(line_marker)
            
            # Create waypoint spheres
            for i, wp in enumerate(waypoints):
                sphere = Marker()
                sphere.header.frame_id = 'map'
                sphere.header.stamp = self.get_clock().now().to_msg()
                sphere.ns = 'waypoints'
                sphere.id = i + 1
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                
                sphere.pose.position.x = wp['position']['x']
                sphere.pose.position.y = wp['position']['y']
                sphere.pose.position.z = 0.1
                
                sphere.scale.x = 0.15
                sphere.scale.y = 0.15
                sphere.scale.z = 0.15
                
                # Color gradient based on position in path
                ratio = i / len(waypoints)
                sphere.color.r = 1.0 - ratio
                sphere.color.g = ratio
                sphere.color.b = 0.3
                sphere.color.a = 1.0
                
                markers.markers.append(sphere)
                
                # Add text labels
                text = Marker()
                text.header.frame_id = 'map'
                text.header.stamp = self.get_clock().now().to_msg()
                text.ns = 'waypoint_labels'
                text.id = i + 1000
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                
                text.pose.position.x = wp['position']['x']
                text.pose.position.y = wp['position']['y']
                text.pose.position.z = 0.3
                
                text.scale.z = 0.2
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.color.a = 1.0
                
                text.text = f"#{i+1}"
                markers.markers.append(text)
            
            self.marker_pub.publish(markers)
            
        except Exception as e:
            self.get_logger().warn(f'Failed to load waypoints: {e}')

def main():
    if len(sys.argv) < 2:
        print("❌ Usage: python3 visualize_waypoints.py <waypoint_file.yaml>")
        return
    
    rclpy.init()
    
    waypoint_file = sys.argv[1]
    node = WaypointVisualizer(waypoint_file)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

