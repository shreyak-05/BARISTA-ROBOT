#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import numpy as np
import sys



class Joint2MoverNode(Node):
    def __init__(self):
        super().__init__('joint_2_mover_node')
        # Replace '/position_controllers/command' with your topic
        self.publisher_2 = self.create_publisher(Float64MultiArray, '/position_controller_2/commands', 10)
        self.publisher_3 = self.create_publisher(Float64MultiArray, '/position_controller_3/commands', 10)
        self.publisher_4 = self.create_publisher(Float64MultiArray, '/position_controller_4/commands', 10)
        self.publisher_5 = self.create_publisher(Float64MultiArray, '/position_controller_5/commands', 10)
        self.publisher_6 = self.create_publisher(Float64MultiArray, '/position_controller_6/commands', 10)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.duration = 1.57  # Total time to move from 0 to pi
        self.target_angle = (math.pi)/2
        self.get_logger().info("Joint Mover Node has started.")
    
    def timer_callback(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_time > self.duration:
            elapsed_time = self.duration

        # Linear interpolation
        target_position = (elapsed_time / self.duration) * self.target_angle

        msg2 = Float64MultiArray()
        msg3 = Float64MultiArray()
        msg4 = Float64MultiArray()
        msg5 = Float64MultiArray()
        msg6 = Float64MultiArray()

        # Set data for all joints controlled by the topic
        msg2.data = [target_position]  # Adjust size if multiple joints
        msg3.data = [0.0]
        msg4.data = [0.0]
        msg5.data = [0.0]
        msg6.data = [0.0]



        self.publisher_2.publish(msg2)
        self.publisher_3.publish(msg3)
        self.publisher_4.publish(msg4)
        self.publisher_5.publish(msg5)
        self.publisher_6.publish(msg6)
        
        self.get_logger().info(f'Published positions: {msg2.data}')

        if elapsed_time >= self.duration:
            self.get_logger().info("Motion complete. Stopping timer.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = Joint2MoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

