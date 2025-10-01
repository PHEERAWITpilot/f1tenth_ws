#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import time
import re

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')
        
        # Declare parameters (IMU only)
        self.declare_parameter('imu_serial_port', '/dev/imu')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('enable_debug_logging', False)
        
        # Get parameters
        self.imu_port = self.get_parameter('imu_serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self.debug_logging = self.get_parameter('enable_debug_logging').value
        
        # Publishers (IMU only)
        self.imu_yaw_pub = self.create_publisher(Float64, 'imu/yaw', 10)
        
        # Optional: Additional data publishers (uncomment if needed)
        # self.imu_roll_pub = self.create_publisher(Float64, 'imu/roll', 10)
        # self.imu_pitch_pub = self.create_publisher(Float64, 'imu/pitch', 10)
        
        # Initialize serial connection
        self.imu_serial = None
        self.connect_imu()
        
        # Timer for reading serial data
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.read_serial_data)
        
        # Variables for data processing
        self.last_yaw = 0.0
        self.last_roll = 0.0
        self.last_pitch = 0.0
        self.data_count = 0
        
        # Regex patterns for parsing Arduino data
        self.yaw_pattern = re.compile(r'Y\(([+-]?\d*\.?\d+)\)')
        self.euler_pattern = re.compile(r'E\(([+-]?\d*\.?\d+),([+-]?\d*\.?\d+),([+-]?\d*\.?\d+)\)')
        
        self.get_logger().info(f"IMU Bridge Node started at {publish_rate}Hz")

    def connect_imu(self):
        """Initialize IMU serial connection"""
        try:
            self.imu_serial = serial.Serial(
                port=self.imu_port,
                baudrate=self.baud_rate,
                timeout=0.1,
                write_timeout=0.1
            )
            time.sleep(2)  # Allow Arduino to reset
            self.get_logger().info(f"✅ Connected to IMU at {self.imu_port} ({self.baud_rate} baud)")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect to IMU at {self.imu_port}: {e}")
            self.imu_serial = None
        except Exception as e:
            self.get_logger().error(f"❌ Unexpected error connecting to IMU: {e}")
            self.imu_serial = None

    def read_serial_data(self):
        """Read data from IMU serial port and publish to ROS topics"""
        if self.imu_serial and self.imu_serial.is_open:
            self.read_imu_data()

    def read_imu_data(self):
        """Read and process IMU data in your Arduino's format"""
        try:
            while self.imu_serial.in_waiting > 0:
                line = self.imu_serial.readline().decode('utf-8').strip()
                
                if line.startswith('IMU:'):
                    # Parse your Arduino's specific data format
                    # Format: IMU:timestamp,Q(...),E(roll,pitch,yaw),A(...),G(...),M(...),Y(yaw)
                    
                    data_part = line[4:]  # Remove 'IMU:' prefix
                    
                    # Method 1: Extract yaw from Y(value) - most reliable for your format
                    yaw_match = self.yaw_pattern.search(data_part)
                    if yaw_match:
                        yaw = float(yaw_match.group(1))
                        self.last_yaw = yaw
                        
                        # Publish IMU yaw
                        yaw_msg = Float64()
                        yaw_msg.data = self.last_yaw
                        self.imu_yaw_pub.publish(yaw_msg)
                        
                        self.data_count += 1
                        
                        # Log data periodically (every 50 messages to avoid spam)
                        if self.debug_logging and (self.data_count % 50 == 0):
                            self.get_logger().info(f"IMU Yaw: {self.last_yaw:.2f}° (Count: {self.data_count})")
                    
                    # Optional: Extract roll, pitch from E(roll,pitch,yaw) format
                    # euler_match = self.euler_pattern.search(data_part)
                    # if euler_match:
                    #     roll = float(euler_match.group(1))
                    #     pitch = float(euler_match.group(2))
                    #     yaw_from_euler = float(euler_match.group(3))
                    #     
                    #     self.last_roll = roll
                    #     self.last_pitch = pitch
                    #     
                    #     # Publish additional data if publishers are enabled
                    #     # roll_msg = Float64()
                    #     # roll_msg.data = roll
                    #     # self.imu_roll_pub.publish(roll_msg)
                    #     
                    #     # pitch_msg = Float64()
                    #     # pitch_msg.data = pitch
                    #     # self.imu_pitch_pub.publish(pitch_msg)
                    
                    # Debug: Print raw data occasionally
                    if self.debug_logging and (self.data_count % 100 == 0):
                        self.get_logger().debug(f"Raw IMU data: {data_part[:100]}...")
                        
        except UnicodeDecodeError as e:
            self.get_logger().warn(f"IMU Unicode decode error: {e}")
        except ValueError as e:
            self.get_logger().warn(f"IMU value parsing error: {e}")
        except Exception as e:
            self.get_logger().error(f"IMU read error: {e}")

    def get_status(self):
        """Return current status of IMU connection"""
        imu_status = "✅ Connected" if (self.imu_serial and self.imu_serial.is_open) else "❌ Disconnected"
        return {
            'imu': imu_status,
            'last_yaw': self.last_yaw,
            'data_count': self.data_count
        }

    def destroy_node(self):
        """Clean up serial connection on shutdown"""
        self.get_logger().info("Shutting down IMU Bridge...")
        
        if self.imu_serial and self.imu_serial.is_open:
            self.imu_serial.close()
            self.get_logger().info("IMU serial connection closed")
        
        # Print final statistics
        self.get_logger().info(f"Total IMU messages processed: {self.data_count}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArduinoBridge()
        
        # Print initial status
        status = node.get_status()
        node.get_logger().info(f"Node Status - IMU: {status['imu']}")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("IMU Bridge shutdown complete.")

if __name__ == '__main__':
    main()

