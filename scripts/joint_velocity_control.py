#!/usr/bin/env python3

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import Float64MultiArray # type: ignore
from sensor_msgs.msg import JointState # type: ignore
from sympy import symbols, sin, cos, Matrix, Derivative, pi, simplify, init_printing
import math
import numpy as np
import sys







class JointVelocityNode(Node):
    def __init__(self):
        super().__init__('joint_velocity_node')
        self.get_logger().info("Joint Velocity Node has started.")

        init_printing(use_unicode=True)

        # Initializing Symbols
        a_1, a_2, a_3, a_4, a_5, a_6, =  symbols('a_1 a_2 a_3 a_4 a_5 a_6')
        alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6 = symbols('alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6')
        d_1, d_2, d_3, d_4, d_5, d_6 = symbols('d_1 d_2 d_3 d_4 d_5 d_6')
        theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

        self.get_logger().info("Initialized Symbols")

        # Initialize DH Parameters
        a_1 = 0
        a_2 = -.6127
        a_3 = -.57155
        a_4 = 0
        a_5 = 0
        a_6 = 0

        alpha_1 = pi/2
        alpha_2 = 0
        alpha_3 = 0
        alpha_4 = pi/2
        alpha_5 = pi/2
        alpha_6 = 0

        d_1 = .1807
        d_2 = 0
        d_3 = 0
        d_4 = 0.17415
        d_5 = .11985
        d_6 = .11655


        # Constructing Matrices

        A_1 = Matrix([[cos(theta_1 - pi), -cos(alpha_1)*sin(theta_1 - pi), sin(alpha_1)*sin(theta_1 - pi), a_1*cos(theta_1 - pi)],
             [sin(theta_1 - pi), cos(alpha_1)*cos(theta_1 - pi), -sin(alpha_1)*cos(theta_1 - pi), a_1*sin(theta_1 - pi)],
             [0, sin(alpha_1), cos(alpha_1), d_1],
             [0, 0, 0, 1]])

        A_2 = Matrix([[cos(theta_2 - (pi/2)), -cos(alpha_2)*sin(theta_2 - (pi/2)), sin(alpha_2)*sin(theta_2 - (pi/2)), a_2*cos(theta_2 - (pi/2))],
             [sin(theta_2 - (pi/2)), cos(alpha_2)*cos(theta_2 - (pi/2)), -sin(alpha_2)*cos(theta_2 - (pi/2)), a_2*sin(theta_2 - (pi/2))],
             [0, sin(alpha_2), cos(alpha_2), d_2],
             [0, 0, 0, 1]])
        A_3 = Matrix([[cos(theta_3), -cos(alpha_3)*sin(theta_3), sin(alpha_3)*sin(theta_3), a_3*cos(theta_3)],
             [sin(theta_3), cos(alpha_3)*cos(theta_3), -sin(alpha_3)*cos(theta_3), a_3*sin(theta_3)],
             [0, sin(alpha_3), cos(alpha_3), d_3],
             [0, 0, 0, 1]])

        A_4 = Matrix([[cos(theta_4 - (pi/2)), -cos(alpha_4)*sin(theta_4 - (pi/2)), sin(alpha_4)*sin(theta_4 - (pi/2)), a_4*cos(theta_4 - (pi/2))],
             [sin(theta_4 - (pi/2)), cos(alpha_4)*cos(theta_4 - (pi/2)), -sin(alpha_4)*cos(theta_4 - (pi/2)), a_4*sin(theta_4 - (pi/2))],
             [0, sin(alpha_4), cos(alpha_4), d_4],
             [0, 0, 0, 1]])

        A_5 = Matrix([[cos(theta_5), -cos(alpha_5)*sin(theta_5), sin(alpha_5)*sin(theta_5), a_5*cos(theta_5)],
             [sin(theta_5), cos(alpha_5)*cos(theta_5), -sin(alpha_5)*cos(theta_5), a_5*sin(theta_5)],
             [0, sin(alpha_5), cos(alpha_5), d_5],
             [0, 0, 0, 1]])

        A_6 = Matrix([[cos(theta_6), -cos(alpha_6)*sin(theta_6), sin(alpha_6)*sin(theta_6), a_6*cos(theta_6)],
             [sin(theta_6), cos(alpha_6)*cos(theta_6), -sin(alpha_6)*cos(theta_6), a_6*sin(theta_6)],
             [0, sin(alpha_6), cos(alpha_6), d_6],
             [0, 0, 0, 1]])

        self.get_logger().info("Constructed Matrices")


        self.get_logger().info("Subbed Values")

        # Calculating Transformations
        A_1_wrt_0 = A_1

        A_2_wrt_0 = A_1_wrt_0 * A_2

        A_3_wrt_0 = A_2_wrt_0 * A_3

        A_4_wrt_0 = A_3_wrt_0 * A_4

        A_5_wrt_0 = A_4_wrt_0 * A_5

        A_6_wrt_0 = simplify(A_5_wrt_0 * A_6)

        #pretty_formula = pretty(A_6_wrt_0)

        self.get_logger().info("Calculated Transforms")

        #self.get_logger().info(f"6 wrt 0 Transformation Matrix:\n{pretty_formula}")

        # Collecting Z Values

        Z_1 = Matrix([0, 0, 1])

        Z_2 = Matrix(A_1_wrt_0.col(2)[:3])

        Z_3 = Matrix(A_2_wrt_0.col(2)[:3])

        Z_4 = Matrix(A_3_wrt_0.col(2)[:3])

        Z_5 = Matrix(A_4_wrt_0.col(2)[:3])

        Z_6 = Matrix(A_5_wrt_0.col(2)[:3])

        # Collecting P Value of Final Transformation

        P_6_wrt_0 = Matrix(A_6_wrt_0.col(3)[:3])

        x = P_6_wrt_0[0]
        y = P_6_wrt_0[1]
        z = P_6_wrt_0[2]

        # Calculating Partial Derivatives

        q_1_x_dot = Derivative(x, theta_1).doit()
        q_2_x_dot = Derivative(x, theta_2).doit()
        q_3_x_dot = Derivative(x, theta_3).doit()
        q_4_x_dot = Derivative(x, theta_4).doit()
        q_5_x_dot = Derivative(x, theta_5).doit()
        q_6_x_dot = Derivative(x, theta_6).doit()

        q_1_y_dot = Derivative(y, theta_1).doit()
        q_2_y_dot = Derivative(y, theta_2).doit()
        q_3_y_dot = Derivative(y, theta_3).doit()
        q_4_y_dot = Derivative(y, theta_4).doit()
        q_5_y_dot = Derivative(y, theta_5).doit()
        q_6_y_dot = Derivative(y, theta_6).doit()

        q_1_z_dot = Derivative(z, theta_1).doit()
        q_2_z_dot = Derivative(z, theta_2).doit()
        q_3_z_dot = Derivative(z, theta_3).doit()
        q_4_z_dot = Derivative(z, theta_4).doit()
        q_5_z_dot = Derivative(z, theta_5).doit()
        q_6_z_dot = Derivative(z, theta_6).doit()
                
        # Constructing Jacobian

        J = Matrix([[q_1_x_dot, q_2_x_dot, q_3_x_dot, q_4_x_dot, q_5_x_dot, q_6_x_dot],
                    [q_1_y_dot, q_2_y_dot, q_3_y_dot, q_4_y_dot, q_5_y_dot, q_6_y_dot],
                    [q_1_z_dot, q_2_z_dot, q_3_z_dot, q_4_z_dot, q_5_z_dot, q_6_z_dot],
                    [Z_1, Z_2, Z_3, Z_4, Z_5, Z_6]])

        self.get_logger().info("Computed J")

        theta_1_vals = [0]
        theta_2_vals = [0.1]
        theta_3_vals = [-.383]
        theta_4_vals = [0.283]
        theta_5_vals = [0.0001]
        theta_6_vals = [0]

        J_sub = J.subs({theta_1: theta_1_vals[0], theta_2: theta_2_vals[0], theta_3: theta_3_vals[0], theta_4: theta_4_vals[0], theta_5: theta_5_vals[0], theta_6: theta_6_vals[0]})
        self.get_logger().info(f"6 wrt 0 Transformation Matrix:\n{J_sub}")
    
        # Creating Publishers
        self.joint_velocity_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        self.effort_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.joint_names = ['link_1_joint', 'link_2_joint', 'link_3_joint', 'link_4_joint', 'link_5_joint', 'link_6_joint','left_finger_1_joint', 'left_finger_2_joint', 'right_finger_1_joint', 'right_finger_2_joint']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_efforts = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Creating Subscriber
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_joint_states)
        #self.start_time = self.get_clock().now()
        #self.duration = 5  # Total time to move from 0 to pi
        #self.target_angle = math.pi

    def publish_joint_states(self):
        """Publishes joint states with effort values."""
        joint_state_msg = JointState()

        #self.get_logger().info("Publishing Effort")

        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        joint_state_msg.velocity = self.joint_velocities  # Optional: you can leave it empty or set to zero
        joint_state_msg.effort = self.joint_efforts  # Set effort values here

        self.effort_pub.publish(joint_state_msg)
        self.get_logger().info("Publishing Effort")


    def joint_state_callback(self, msg):
        """Callback to process received joint state messages."""


        joint_1_position = None
        joint_2_position = None
        joint_3_position = None
        joint_4_position = None
        joint_5_position = None
        joint_6_position = None
        lf1_position = None
        lf2_position = None
        rf1_position = None
        rf2_position = None

        index_1 = msg.name.index('link_1_joint')
        index_2 = msg.name.index('link_2_joint')
        index_3 = msg.name.index('link_3_joint')
        index_4 = msg.name.index('link_4_joint')
        index_5 = msg.name.index('link_5_joint')
        index_6 = msg.name.index('link_6_joint')
        lf1_index = msg.name.index("left_finger_1_joint")
        lf2_index = msg.name.index("left_finger_2_joint")
        rf1_index = msg.name.index("right_finger_1_joint")
        rf1_index = msg.name.index("right_finger_2_joint")




        joint_1_position = msg.position[index_1]
        joint_2_position = msg.position[index_2]
        joint_3_position = msg.position[index_3]
        joint_4_position = msg.position[index_4]
        joint_5_position = msg.position[index_5]
        joint_6_position = msg.position[index_6]




        #if joint_1_position is not None:
        #    self.get_logger().info(f"Joint 1 position: {joint_1_position}")
        #if joint_2_position is not None:
        #    self.get_logger().info(f"Joint 2 position: {joint_2_position}")
        #if joint_3_position is not None:
        #    self.get_logger().info(f"Joint 3 position: {joint_3_position}")
        #if joint_4_position is not None:
        #    self.get_logger().info(f"Joint 4 position: {joint_4_position}")
        #if joint_5_position is not None:
        #    self.get_logger().info(f"Joint 5 position: {joint_5_position}")
        #if joint_6_position is not None:
        #    self.get_logger().info(f"Joint 6 position: {joint_6_position}")
        #else:
        #    self.get_logger().warn("Joint 1 position not found!")


    #def timer_callback(self):
    #    elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
    #    if elapsed_time > self.duration:
    #        elapsed_time = self.duration

        # Linear interpolation
    #    target_position = (elapsed_time / self.duration) * self.target_angle

    #    msg = Float64MultiArray()
        # Set data for all joints controlled by the topic
    #    msg.data = [target_position]  # Adjust size if multiple joints
    #    self.publisher_.publish(msg)
    #    self.get_logger().info(f'Published positions: {msg.data}')

    #    if elapsed_time >= self.duration:
    #        self.get_logger().info("Motion complete. Stopping timer.")
    #        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    velocity_node = JointVelocityNode()
    rclpy.spin(velocity_node)
    velocity_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()