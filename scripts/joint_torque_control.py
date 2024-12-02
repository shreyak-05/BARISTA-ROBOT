#!/usr/bin/env python3

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import Float64MultiArray # type: ignore
from sensor_msgs.msg import JointState # type: ignore
from sympy import symbols, sin, cos, Matrix, Derivative, pi, simplify, init_printing
import math
import numpy as np
import sys







class JointTorqueNode(Node):
    def __init__(self):
        super().__init__('joint_torque_node')
        self.get_logger().info("Joint Torque Node has started.")

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


        

        m1 = 3.50205715504707
        m2 = 5.85231057454819
        m3 = 4.62234527371938
        m4 = 0.952056234394498
        m5 = 0.852888399213539
        m6 = 0.350250328419854

        g = 9.81

        p1 = m1 * g * (d_1/2)
        p2 = m2 * g * (d_1 + ((-a_2)/2)*cos(theta_2))
        p3 = m3 * g * (d_1 + (-a_2)*cos(theta_2) + ((-a_3)/2)*cos(theta_2 + theta_3))
        p4 = m4 * g * (d_1 + (-a_2)*cos(theta_2) + (-a_3)*cos(theta_2 + theta_3))
        p5 = m5 * g * (d_1 + (-a_2)*cos(theta_2) + (-a_3)*cos(theta_2 + theta_3) + (d_5/2)*cos(theta_2 + theta_3 + theta_4))
        p6 = m6 * g * (d_1 + (-a_2)*cos(theta_2) + (-a_3)*cos(theta_2 + theta_3) + d_5*cos(theta_2 + theta_3 + theta_4))

        p_sum = simplify((p1 + p2 + p3 + p4 + p5 + p6)*(-1))

        L1_dot = (Derivative(p_sum, theta_1).doit()) * (-1)
        L2_dot = (Derivative(p_sum, theta_2).doit()) * (-1)
        L3_dot = (Derivative(p_sum, theta_3).doit()) * (-1)
        L4_dot = (Derivative(p_sum, theta_4).doit()) * (-1)
        L5_dot = (Derivative(p_sum, theta_5).doit()) * (-1)
        L6_dot = (Derivative(p_sum, theta_6).doit()) * (-1)

        # Gravity Matrix

        g_matrix = Matrix([L1_dot,
                            L2_dot,
                            L3_dot,
                            L4_dot,
                            L5_dot,
                            L6_dot])
        

        self.T = g_matrix
        #self.get_logger().info(self.T)
        self.get_logger().info("Computed g matrix")


        # Creating Publishers
        #self.joint_velocity_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_effort_pub = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        
        # Creating Subscriber
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        """Callback to process received joint state messages."""
        theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

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


        t_vals = self.T.subs({theta_1: joint_1_position, theta_2: joint_2_position, theta_3: joint_3_position, theta_4: joint_4_position, theta_5: joint_5_position, theta_6: joint_6_position})
        float_t_vals = list(map(float,t_vals))

        self.get_logger().info('subbed t_vals')
        self.get_logger().info(f"subbed t_vals:\n{float_t_vals}")
        #self.get_logger().info(t_vals)
        j_effort = Float64MultiArray()
        j_effort.data = [float_t_vals[0], float_t_vals[1], float_t_vals[2], float_t_vals[3], float_t_vals[4], float_t_vals[5], float_t_vals[5], float_t_vals[5], float_t_vals[5], float_t_vals[5]]

        self.joint_effort_pub.publish(j_effort)
        self.get_logger().info('publish effort')

        

def main(args=None):
    rclpy.init(args=args)
    torque_node = JointTorqueNode()
    rclpy.spin(torque_node)
    torque_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()