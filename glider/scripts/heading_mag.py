#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import math


class MagnetometerHeadingNode(Node):
    def __init__(self):
        super().__init__('magnetometer_heading_node')
        
        # Create subscriber for magnetometer data
        self.subscription = self.create_subscription(
            MagneticField,
            '/vectornav/magnetic',  # Change this to your magnetometer topic name
            self.magnetometer_callback,
            10
        )
        
        self.get_logger().info('Magnetometer heading node started')
        self.get_logger().info('Subscribing to topic: /imu/mag')

    def magnetometer_callback(self, msg):
        """
        Callback function to process magnetometer data and calculate heading
        """
        # Extract magnetic field components
        mag_x = msg.magnetic_field.x
        mag_y = msg.magnetic_field.y
        mag_z = msg.magnetic_field.z
        
        # Calculate heading using atan2 (yaw angle)
        # In ENU frame: heading = atan2(mag_y, mag_x)
        # This gives heading relative to magnetic north
        heading_rad = math.atan2(mag_y, mag_x)
        
        # Convert to degrees
        heading_deg = math.degrees(heading_rad)
        
        # Normalize to 0-360 degrees
        if heading_deg < 0:
            heading_deg += 360.0
        
        # Calculate magnetic field magnitude
        magnitude = math.sqrt(mag_x**2 + mag_y**2 + mag_z**2)
        
        # Print the results
        self.get_logger().info(
            f'Heading: {heading_deg:.2f}° | Magnitude: {magnitude:.3f} µT'
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        magnetometer_node = MagnetometerHeadingNode()
        rclpy.spin(magnetometer_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        if 'magnetometer_node' in locals():
            magnetometer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
