#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped


class ImuOrientationPublisher(Node):
    def __init__(self):
        super().__init__('imu_orientation_publisher')
        
        # Create subscriber to IMU topic
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/vectornav/imu',  # Change this to your IMU topic name
            self.imu_callback,
            10
        )
        
        # Create publisher for pose with orientation only
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/orientation_pose',
            10
        )
        
        self.get_logger().info('IMU Orientation Publisher node started')
        self.get_logger().info('Subscribing to: /imu/data')
        self.get_logger().info('Publishing to: /orientation_pose')

    def imu_callback(self, msg):
        """
        Callback function for IMU messages.
        Extracts orientation and publishes as PoseStamped with zero position.
        """
        # Create PoseStamped message
        pose_msg = PoseStamped()
        
        # Set header information
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = msg.header.frame_id
        
        # Set position to (0, 0, 0)
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        
        # Copy orientation from IMU message
        pose_msg.pose.orientation.x = msg.orientation.x
        pose_msg.pose.orientation.y = msg.orientation.y
        pose_msg.pose.orientation.z = msg.orientation.z
        pose_msg.pose.orientation.w = msg.orientation.w
        
        # Publish the pose
        self.pose_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = ImuOrientationPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down IMU Orientation Publisher node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
