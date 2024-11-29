#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
import sys



class JointMoverNode(Node):
    def __init__(self):
        super().__init__('joint_mover_node')
        # Replace '/position_controllers/command' with your topic
        self.publisher_ = self.create_publisher(Float64MultiArray, '/position_controller_1/commands', 10)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.duration = 3.14  # Total time to move from 0 to pi
        self.target_angle = math.pi
        self.get_logger().info("Joint Mover Node has started.")
    
    def timer_callback(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_time > self.duration:
            elapsed_time = self.duration

        # Linear interpolation
        target_position = (elapsed_time / self.duration) * self.target_angle

        msg = Float64MultiArray()
        # Set data for all joints controlled by the topic
        msg.data = [target_position]  # Adjust size if multiple joints
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published positions: {msg.data}')

        if elapsed_time >= self.duration:
            self.get_logger().info("Motion complete. Stopping timer.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = JointMoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

