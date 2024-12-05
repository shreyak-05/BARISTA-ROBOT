#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sympy import Matrix, symbols, cos, sin, pi
import numpy as np


class VelocityControlRobot(Node):
    def __init__(self):
        super().__init__('velocity_control_robot')

        # Velocity publishers
        self.pub_shoulder = self.create_publisher(Float64, '/shoulder_pan_joint_velocity_controller/command', 10)
        self.pub_upperarm = self.create_publisher(Float64, '/shoulder_lift_joint_velocity_controller/command', 10)
        self.pub_elbow = self.create_publisher(Float64, '/elbow_joint_velocity_controller/command', 10)
        self.pub_wrist1 = self.create_publisher(Float64, '/wrist_1_joint_velocity_controller/command', 10)
        self.pub_wrist2 = self.create_publisher(Float64, '/wrist_2_joint_velocity_controller/command', 10)
        self.pub_wrist3 = self.create_publisher(Float64, '/wrist_3_joint_velocity_controller/command', 10)

        # Joint angles
        self.q_vals = [0, 0.1, -0.383, 0.283, 0.0001, 0]

        # Create a timer for the control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Define trajectory in task space (example: constant velocity along x-axis)
        self.trajectory_x_dot = [0.01] * 100
        self.trajectory_y_dot = [0] * 100
        self.trajectory_z_dot = [0] * 100
        self.current_step = 0

    def control_loop(self):
        if self.current_step >= len(self.trajectory_x_dot):
            self.get_logger().info('Trajectory completed.')
            self.destroy_timer(self.timer)
            return

        # Task-space velocities
        x_dot = self.trajectory_x_dot[self.current_step]
        y_dot = self.trajectory_y_dot[self.current_step]
        z_dot = self.trajectory_z_dot[self.current_step]

        # Compute end-effector velocity
        x_vals = Matrix([x_dot, y_dot, z_dot, 0, 0, 0])

        # Compute Jacobian
        J = self.compute_jacobian(self.q_vals)
        inv_J = J.pinv()  # Pseudo-inverse of Jacobian

        # Compute joint velocities
        q_dot = inv_J * x_vals

        # Publish joint velocities
        self.pub_shoulder.publish(Float64(data=float(q_dot[0])))
        self.pub_upperarm.publish(Float64(data=float(q_dot[1])))
        self.pub_elbow.publish(Float64(data=float(q_dot[2])))
        self.pub_wrist1.publish(Float64(data=float(q_dot[3])))
        self.pub_wrist2.publish(Float64(data=float(q_dot[4])))
        self.pub_wrist3.publish(Float64(data=float(q_dot[5])))

        # Update joint positions
        self.q_vals = [q + float(q_dot[i]) * 0.1 for i, q in enumerate(self.q_vals)]  # Euler integration

        self.current_step += 1

    @staticmethod
    def compute_jacobian(joint_angles):
        # DH Parameters
        a = [0, -0.73731, -0.3878, 0, 0, 0]
        alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]
        d = [0.1833, 0, 0, 0.0955, 0.1155, 0.1218]
        theta = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

        # DH Transformation matrix
        def dh_transform(a, alpha, d, theta):
            return Matrix([
                [cos(theta), -cos(alpha) * sin(theta), sin(alpha) * sin(theta), a * cos(theta)],
                [sin(theta), cos(alpha) * cos(theta), -sin(alpha) * cos(theta), a * sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]
            ])

        # Forward kinematics
        def forward_kinematics(joint_angles):
            T = Matrix.eye(4)  # Start with identity matrix
            for i, angle in enumerate(joint_angles):
                T *= dh_transform(a[i], alpha[i], d[i], angle)
            return T

        # Compute the full transformation matrix
        T_6_0 = forward_kinematics(joint_angles)
        P = T_6_0[:3, 3]  # Extract position from the final transformation matrix
        Z = [Matrix([0, 0, 1])]  # Base frame z-axis

        # Compute intermediate transformations and Jacobian columns
        jacobian = []
        T_prev = Matrix.eye(4)
        for i in range(len(joint_angles)):
            Z_i = T_prev[:3, 2]  # Z-axis of the current frame
            P_i = T_prev[:3, 3]  # Position of the current frame
            J_v = Z_i.cross(P - P_i)  # Linear velocity
            J_w = Z_i  # Angular velocity
            jacobian.append(Matrix.vstack(J_v, J_w))
            T_prev *= dh_transform(a[i], alpha[i], d[i], joint_angles[i])

        return Matrix.hstack(*jacobian)


def main(args=None):
    rclpy.init(args=args)

    robot_controller = VelocityControlRobot()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass

    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
