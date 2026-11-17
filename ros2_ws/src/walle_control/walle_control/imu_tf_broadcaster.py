#!/usr/bin/env python3
"""
imu_tf_broadcaster.py
=====================
Subscribes to /imu/data published by ESP32 via micro-ROS and broadcasts
the dynamic transform from 'odom' (or 'world') to 'base_link'.

Why this is needed:
  In RViz, if Fixed Frame is 'base_link', the robot model will always stay
  upright relative to the camera because the camera is locked to the robot.
  With 'odom' as Fixed Frame and this node broadcasting odom -> base_link
  with the IMU orientation, tilting the real robot causes the 3D model in RViz
  to visibly tilt forward and backward in real time!
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.get_logger().info('IMU TF Broadcaster started: broadcasting odom -> base_link')

    def imu_callback(self, msg: Imu):
        t = TransformStamped()
        # Use PC ROS2 clock to guarantee exact synchronization with robot_state_publisher and RViz TF buffer
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Robot stays at center (0, 0, wheel_radius)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.033

        # Dynamic orientation from IMU fused pitch and roll
        t.transform.rotation = msg.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
