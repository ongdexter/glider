#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

class OdomHeadingSubscriber(Node):
    def __init__(self):
        super().__init__('odom_heading_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/glider/odom',
            self.odom_callback,
            10)
        self.get_logger().info('Odometry heading subscriber initialized')

    def odom_callback(self, msg):
        # Extract quaternion from odometry message
        orientation = msg.pose.pose.orientation
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # Convert quaternion to Euler angles using scipy
        rotation = Rotation.from_quat(quat)
        euler = rotation.as_euler('xyz', degrees=False)
        
        # Extract yaw (rotation around z-axis)
        yaw = euler[2]
        
        # Convert radians to degrees
        heading_degrees = yaw * 180.0 / 3.141592653589793
        
        # Print the heading
        self.get_logger().info(f'Heading: {heading_degrees:.2f} degrees')

def main(args=None):
    rclpy.init(args=args)
    node = OdomHeadingSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
