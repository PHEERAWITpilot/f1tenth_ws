#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64
import math
import time
from collections import deque

class VescSpeedCalculator(Node):
    def __init__(self):
        super().__init__('vesc_speed_calculator')
        
        # Parameters (matching your Arduino constants)
        self.declare_parameter('wheel_radius', 0.05)        # 2.5 cm radius (5 cm diameter)
        self.declare_parameter('gear_ratio', 10.88)         # 2.72 × 4 = 10.88 (your 4x fix)
        self.declare_parameter('motor_pole_pairs', 2)       # 4-pole motor = 2 pole pairs
        self.declare_parameter('motor_stop_timeout', 1.0)   # 1000ms timeout
        self.declare_parameter('num_samples', 10)           # Moving average samples
        self.declare_parameter('publish_rate', 50.0)        # 50Hz (20ms interval)
        self.declare_parameter('erpm_deadzone', 5.0)        # ERPM dead zone
        self.declare_parameter('velocity_threshold', 0.01)   # 1cm/s minimum velocity
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.motor_pole_pairs = self.get_parameter('motor_pole_pairs').value
        self.motor_stop_timeout = self.get_parameter('motor_stop_timeout').value
        self.num_samples = self.get_parameter('num_samples').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.erpm_deadzone = self.get_parameter('erpm_deadzone').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        
        # Calculate distance per revolution (equivalent to distancePerCount)
        wheel_circumference = 2 * math.pi * self.wheel_radius
        self.distance_per_revolution = wheel_circumference / self.gear_ratio
        
        # State variables (matching Arduino variables)
        self.x_vel = 0.0
        self.total_distance = 0.0
        self.prev_time = time.time()
        self.prev_erpm = 0.0
        self.first_reading = True
        
        # Moving average filter
        self.x_vel_samples = deque(maxlen=self.num_samples)
        for _ in range(self.num_samples):
            self.x_vel_samples.append(0.0)
        
        # Publishers
        self.velocity_pub = self.create_publisher(Float64, 'vel', 10)
        self.distance_pub = self.create_publisher(Float64, 'distance', 10)
        
        # Subscribe to VESC sensors
        self.vesc_sub = self.create_subscription(
            VescStateStamped,
            'sensors/core',
            self.vesc_callback,
            10
        )
        
        # Timer for publishing at fixed rate (50Hz)
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_data
        )
        
        self.get_logger().info('VESC Speed Calculator Started')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m, Gear ratio: {self.gear_ratio}')
        self.get_logger().info(f'ERPM deadzone: {self.erpm_deadzone}, Velocity threshold: {self.velocity_threshold} m/s')
        self.get_logger().info(f'Distance per rev: {self.distance_per_revolution:.6f}m')

    def vesc_callback(self, msg):
        """Calculate velocity with noise filtering and direction"""
        current_time = time.time()
        
        # ✅ Use signed ERPM to get direction
        current_erpm_signed = msg.state.speed  # Keep the sign for direction
        current_erpm_abs = abs(current_erpm_signed)  # Absolute value for calculations
        
        # ERPM dead zone - ignore readings below threshold
        if current_erpm_abs < self.erpm_deadzone:
            self.x_vel = 0.0
            self.apply_moving_average()
            return
        
        # ✅ Determine direction from ERPM sign
        direction = 1.0 if current_erpm_signed >= 0 else -1.0
        
        # Convert ERPM to mechanical RPM (use absolute value for calculation)
        mechanical_rpm = current_erpm_abs / self.motor_pole_pairs
        
        if self.first_reading:
            # First reading - initialize
            self.first_reading = False
            self.x_vel = 0.0
            self.prev_erpm = current_erpm_signed  # Store signed value
            self.prev_time = current_time
            self.get_logger().info("First VESC reading detected")
        else:
            time_delta = current_time - self.prev_time
            
            # Only calculate if we have a reasonable time delta
            if time_delta > 0.001:  # Minimum 1ms between calculations
                
                # If motor is spinning
                if mechanical_rpm > 0:
                    # Calculate velocity from RPM
                    rps = mechanical_rpm / 60.0
                    velocity_magnitude = rps * self.distance_per_revolution
                    
                    # ✅ Apply direction to velocity
                    self.x_vel = velocity_magnitude * direction
                    
                    # Update total distance (integrate absolute distance)
                    distance_increment = abs(self.x_vel) * time_delta
                    self.total_distance += distance_increment
                    
                else:
                    # Motor stopped or very slow
                    self.x_vel = 0.0
                
                # Timeout detection
                if time_delta >= self.motor_stop_timeout:
                    self.x_vel = 0.0
                    self.get_logger().debug("Motor stop timeout triggered")
        
        # Update previous values
        self.prev_erpm = current_erpm_signed  # Store signed value
        self.prev_time = current_time
        
        # Apply moving average filter
        self.apply_moving_average()

    def apply_moving_average(self):
        """Apply moving average filter with NaN protection and velocity threshold"""
        # NaN protection
        if math.isnan(self.x_vel) or math.isinf(self.x_vel):
            self.x_vel = 0.0
        
        # Update moving average (circular buffer)
        self.x_vel_samples.append(self.x_vel)
        
        # Calculate average (preserves sign)
        raw_average = sum(self.x_vel_samples) / len(self.x_vel_samples)
        
        # Apply velocity threshold - ignore velocities below threshold
        if abs(raw_average) < self.velocity_threshold:
            self.x_vel = 0.0
        else:
            self.x_vel = raw_average

    def publish_data(self):
        """Publish data at fixed rate (matching Arduino 20ms/50Hz)"""
        # ✅ Publish signed velocity (with direction)
        vel_msg = Float64()
        vel_msg.data = self.x_vel  # Now includes sign for direction
        self.velocity_pub.publish(vel_msg)
        
        # Publish distance
        dist_msg = Float64()
        dist_msg.data = self.total_distance
        self.distance_pub.publish(dist_msg)
        
        # Enhanced debug output showing direction
        direction_str = "FWD" if self.x_vel > 0 else "REV" if self.x_vel < 0 else "STOP"
        self.get_logger().debug(
            f'VESC:{{\"vel\":{self.x_vel:.4f},\"dist\":{self.total_distance:.4f},\"dir\":\"{direction_str}\"}}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VescSpeedCalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

