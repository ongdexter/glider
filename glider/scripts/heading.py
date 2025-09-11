#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math


class RpyPrinter(Node):
    def __init__(self):
        super().__init__('rpy_printer')
        
        # Create subscriber to IMU topic
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/VN100T/imu',  # Subscribe directly to IMU topic
            self.imu_callback,
            10
        )
        
        # Create timer for 2Hz printing (0.5 seconds interval)
        self.timer = self.create_timer(0.5, self.print_rpy)
        
        # Store latest orientation data
        self.latest_orientation = None
        self.latest_timestamp = None
        
        self.get_logger().info('Roll Pitch Yaw Printer node started')
        self.get_logger().info('Subscribing to: /imu/data')
        self.get_logger().info('Printing RPY at 2Hz')

    def imu_callback(self, msg):
        """
        Callback function for IMU messages.
        Stores the latest orientation data.
        """
        self.latest_orientation = msg.orientation
        self.latest_timestamp = msg.header.stamp

    def quaternion_to_euler(self, quat):
        """
        Convert quaternion to roll, pitch, yaw (in radians).
        
        Args:
            quat: geometry_msgs.msg.Quaternion
            
        Returns:
            tuple: (roll, pitch, yaw) in radians
        """
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def print_rpy(self):
        """
        Timer callback function that prints roll, pitch, yaw at 2Hz.
        """
        if self.latest_orientation is None:
            self.get_logger().warn('No IMU data received yet')
            return
        
        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(self.latest_orientation)
        
        # Convert to degrees for easier reading
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # Print with formatting
        self.get_logger().info(
            f'Roll: {roll_deg:8.2f}°  Pitch: {pitch_deg:8.2f}°  Yaw: {yaw_deg:8.2f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = RpyPrinter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Roll Pitch Yaw Printer node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
