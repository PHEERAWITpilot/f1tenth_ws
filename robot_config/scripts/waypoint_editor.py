#!/usr/bin/env python3
"""
Interactive Waypoint Editor with Map Background
"""

import cv2
import numpy as np
import yaml
import math

class WaypointEditor:
    def __init__(self, map_yaml_path, waypoints_yaml_path):
        # Load map metadata
        with open(map_yaml_path, 'r') as f:
            self.map_data = yaml.safe_load(f)
        
        # Load map image
        import os
        map_dir = os.path.dirname(map_yaml_path)
        image_path = os.path.join(map_dir, self.map_data['image'])
        
        # Load as grayscale first
        self.map_img_raw = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        
        if self.map_img_raw is None:
            raise Exception(f'Could not load map image: {image_path}')
        
        # Convert to BGR for color display
        self.map_img_raw = cv2.cvtColor(self.map_img_raw, cv2.COLOR_GRAY2BGR)
        
        # Map parameters
        self.resolution = self.map_data['resolution']
        self.origin_x = self.map_data['origin'][0]
        self.origin_y = self.map_data['origin'][1]
        
        # Load waypoints
        self.waypoints_path = waypoints_yaml_path
        try:
            with open(waypoints_yaml_path, 'r') as f:
                wp_data = yaml.safe_load(f)
                self.waypoints = wp_data.get('waypoints', [])
        except:
            print('⚠️  No existing waypoints file, starting fresh!')
            self.waypoints = []
        
        # UI state
        self.selected_wp = None
        self.dragging = False
        self.display_img = None
        
        # Colors
        self.COLOR_WAYPOINT = (0, 255, 0)
        self.COLOR_SELECTED = (0, 255, 255)
        self.COLOR_START = (255, 0, 0)
        self.COLOR_END = (0, 0, 255)
        self.COLOR_TEXT = (255, 255, 255)
        self.COLOR_CONNECTION = (0, 150, 0)
        
        print('🗺️  Map loaded: {}x{} @ {} m/cell'.format(
            self.map_img_raw.shape[1],
            self.map_img_raw.shape[0],
            self.resolution
        ))
        print(f'📍 {len(self.waypoints)} waypoints loaded')
        print('')
        print('🖱️  Controls:')
        print('  Left Click:  Add waypoint')
        print('  Right Click: Delete nearest waypoint')
        print('  Drag:        Move waypoint')
        print('  [s]:         Save waypoints')
        print('  [c]:         Clear all waypoints')
        print('  [q/ESC]:     Quit')
        print('')
    
    def world_to_pixel(self, x, y):
        px = int((x - self.origin_x) / self.resolution)
        py = int(self.map_img_raw.shape[0] - (y - self.origin_y) / self.resolution)
        return px, py
    
    def pixel_to_world(self, px, py):
        x = px * self.resolution + self.origin_x
        y = (self.map_img_raw.shape[0] - py) * self.resolution + self.origin_y
        return x, y
    
    def find_nearest_waypoint(self, px, py, max_dist=30):
        min_dist = max_dist
        nearest_idx = None
        
        for i, wp in enumerate(self.waypoints):
            wx, wy = self.world_to_pixel(wp['position']['x'], wp['position']['y'])
            dist = math.sqrt((wx - px)**2 + (wy - py)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx
    
    def add_waypoint(self, px, py):
        x, y = self.pixel_to_world(px, py)
        
        waypoint = {
            'position': {'x': float(x), 'y': float(y), 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        }
        
        self.waypoints.append(waypoint)
        print(f'✅ Added waypoint {len(self.waypoints)}: ({x:.2f}, {y:.2f})')
    
    def delete_waypoint(self, idx):
        if idx is not None and 0 <= idx < len(self.waypoints):
            wp = self.waypoints[idx]
            print(f'🗑️  Deleted waypoint {idx+1}: ({wp["position"]["x"]:.2f}, {wp["position"]["y"]:.2f})')
            del self.waypoints[idx]
    
    def move_waypoint(self, idx, px, py):
        if idx is not None and 0 <= idx < len(self.waypoints):
            x, y = self.pixel_to_world(px, py)
            self.waypoints[idx]['position']['x'] = float(x)
            self.waypoints[idx]['position']['y'] = float(y)
    
    def save_waypoints(self):
        data = {'waypoints': self.waypoints}
        
        with open(self.waypoints_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        print(f'💾 Saved {len(self.waypoints)} waypoints to {self.waypoints_path}')
    
    def draw(self):
        self.display_img = self.map_img_raw.copy()
        
        # Draw connections
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            
            px1, py1 = self.world_to_pixel(wp1['position']['x'], wp1['position']['y'])
            px2, py2 = self.world_to_pixel(wp2['position']['x'], wp2['position']['y'])
            
            cv2.line(self.display_img, (px1, py1), (px2, py2), 
                    self.COLOR_CONNECTION, 2)
        
        # Draw waypoints
        for i, wp in enumerate(self.waypoints):
            px, py = self.world_to_pixel(wp['position']['x'], wp['position']['y'])
            
            # Choose color
            if i == 0:
                color = self.COLOR_START
            elif i == len(self.waypoints) - 1:
                color = self.COLOR_END
            elif i == self.selected_wp:
                color = self.COLOR_SELECTED
            else:
                color = self.COLOR_WAYPOINT
            
            # Draw circle
            cv2.circle(self.display_img, (px, py), 8, color, -1)
            cv2.circle(self.display_img, (px, py), 10, (255, 255, 255), 2)
            
            # Draw number
            label = str(i + 1)
            cv2.putText(self.display_img, label, (px + 15, py + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.COLOR_TEXT, 2)
        
        # Draw info
        info_text = f'Waypoints: {len(self.waypoints)} | [s] Save | [c] Clear | [q] Quit'
        cv2.putText(self.display_img, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return self.display_img
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            nearest = self.find_nearest_waypoint(x, y)
            if nearest is not None:
                self.selected_wp = nearest
                self.dragging = True
            else:
                self.add_waypoint(x, y)
                self.selected_wp = len(self.waypoints) - 1
        
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.dragging and self.selected_wp is not None:
                self.move_waypoint(self.selected_wp, x, y)
        
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False
        
        elif event == cv2.EVENT_RBUTTONDOWN:
            nearest = self.find_nearest_waypoint(x, y)
            if nearest is not None:
                self.delete_waypoint(nearest)
                self.selected_wp = None
    
    def run(self):
        window_name = 'Waypoint Editor - F1Tenth'
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        while True:
            img = self.draw()
            cv2.imshow(window_name, img)
            
            key = cv2.waitKey(10) & 0xFF
            
            if key == ord('q') or key == 27:
                print('👋 Quitting...')
                break
            elif key == ord('s'):
                self.save_waypoints()
            elif key == ord('c'):
                if len(self.waypoints) > 0:
                    print('🗑️  Clearing all waypoints...')
                    self.waypoints = []
                    self.selected_wp = None
        
        cv2.destroyAllWindows()

def main():
    import sys
    
    # Check arguments
    if len(sys.argv) != 3:
        print('Usage: waypoint_editor.py <map.yaml> <waypoints.yaml>')
        print('')
        print('Example:')
        print('  python3 waypoint_editor.py map.yaml waypoints.yaml')
        sys.exit(1)
    
    map_yaml = sys.argv[1]
    waypoints_yaml = sys.argv[2]
    
    editor = WaypointEditor(map_yaml, waypoints_yaml)
    editor.run()

if __name__ == '__main__':
    main()
