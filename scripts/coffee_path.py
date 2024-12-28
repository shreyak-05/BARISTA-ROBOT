#!/usr/bin/env python3

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import Float64MultiArray # type: ignore
from sympy import symbols, sin, cos, Matrix, Derivative, pi, simplify, init_printing
import math
import numpy as np
import time
from std_msgs.msg import Bool


class CoffeePathNode(Node):
    def __init__(self):
        super().__init__('coffee_path_node')
        self.get_logger().info("coffee path node has started.")

        init_printing(use_unicode=True)

        # Initializing Symbols
        a_1, a_2, a_3, a_4, a_5, a_6, =  symbols('a_1 a_2 a_3 a_4 a_5 a_6')
        alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6 = symbols('alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6')
        d_1, d_2, d_3, d_4, d_5, d_6 = symbols('d_1 d_2 d_3 d_4 d_5 d_6')
        theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

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


        # Calculating Transformations
        A_1_wrt_0 = A_1

        A_2_wrt_0 = A_1_wrt_0 * A_2

        A_3_wrt_0 = A_2_wrt_0 * A_3

        A_4_wrt_0 = A_3_wrt_0 * A_4

        A_5_wrt_0 = A_4_wrt_0 * A_5

        self.A_6_wrt_0 = simplify(A_5_wrt_0 * A_6)


        # Collecting Z Values

        Z_1 = Matrix([0, 0, 1])

        Z_2 = Matrix(A_1_wrt_0.col(2)[:3])

        Z_3 = Matrix(A_2_wrt_0.col(2)[:3])

        Z_4 = Matrix(A_3_wrt_0.col(2)[:3])

        Z_5 = Matrix(A_4_wrt_0.col(2)[:3])

        Z_6 = Matrix(A_5_wrt_0.col(2)[:3])

        # Collecting P Value of Final Transformation

        P_6_wrt_0 = Matrix(self.A_6_wrt_0.col(3)[:3])

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

        self.J = Matrix([[q_1_x_dot, q_2_x_dot, q_3_x_dot, q_4_x_dot, q_5_x_dot, q_6_x_dot],
                    [q_1_y_dot, q_2_y_dot, q_3_y_dot, q_4_y_dot, q_5_y_dot, q_6_y_dot],
                    [q_1_z_dot, q_2_z_dot, q_3_z_dot, q_4_z_dot, q_5_z_dot, q_6_z_dot],
                    [Z_1, Z_2, Z_3, Z_4, Z_5, Z_6]])

        self.J_sub = None

        self.theta_1_vals = 1.57
        self.theta_2_vals = 0.13849
        self.theta_3_vals = 2.38544
        self.theta_4_vals = 0.62393
        self.theta_5_vals = 1.57
        self.theta_6_vals = -math.pi

        
    
        # Creating Publishers
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.gripper_position_pub = self.create_publisher(Float64MultiArray, '/gripper_position_controller/commands', 10)
        # Creating Subscriber
        self.cup_sub = self.create_subscription(Bool, '/cup_topic', self.cup_callback, 10)

        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()

        self.arc1_counter = 0.0
        self.total_duration = 20
        self.target_pose1 = math.pi/2
        self.target_pose2 = math.pi
        self.target_pose3 = math.pi/2

        
        self.counter = 0

        self.cup_val = False

    # Path Plan
    # 1. Move Forward and then Down to inspect serving station
    # 2. If no Cup present, then rotate to cup station, and move forward to pick up cup. Then move backward to previous location
    # 3. Rotate to Coffee Station. Move forward to place cup under coffee dispenser. Move Back to previous location.
    # 4. Rotate to Milk Station. Move forward to place cup under milk dispenser. Move back to previous location.
    # 5. Rotate to serving station. Move forward to place cup on table. Move Back to previous location.

    def cup_callback(self,msg):

        #self. get_logger().info(f'cup state: {msg.data}')
        self.cup_val = msg.data
    def timer_callback(self):

        if self.counter == 0:

            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()
            
            x_vals = Matrix([0.0, 0.50, 0.0 , 0.0, 0.0, 0.0])   

            q_dot = inv_J * x_vals

            self.theta_1_vals += q_dot[0] * self.timer_period
            self.theta_2_vals += q_dot[1] * self.timer_period
            self.theta_3_vals += q_dot[2] * self.timer_period
            self.theta_4_vals += q_dot[3] * self.timer_period
            self.theta_5_vals += q_dot[4] * self.timer_period
            self.theta_6_vals += q_dot[5] * self.timer_period

            j1_theta = float(self.theta_1_vals)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)

            self.j_angle = Float64MultiArray()
            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

            self.end_pos = self.A_6_wrt_0.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})
            
            if self.end_pos[1,3] >= 0.6:
                self.counter += 1
                self.r1 = ((self.end_pos[0,3]) ** 2) + ((self.end_pos[1,3]) ** 2) ** 0.5
                time.sleep(3)
                #self.get_logger().info('sleeping')

                # Check if Cup is on serving table. If yes, then move back to home position and then stop. If No, then make coffee.

            
            #self.get_logger().info(f'counter: {self.counter}')
            self.joint_position_pub.publish(self.j_angle)
            #self.get_logger().info(f'End Effector Position: {self.end_pos[0,3], self.end_pos[1, 3], self.end_pos[2, 3]}')
            #self.get_logger().info(f'Published positions: {self.j_angle.data}')



        if self.counter == 1:

            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()
            
            x_vals = Matrix([0.0, -0.50, 0.0 , 0.0, 0.0, 0.0])   

            q_dot = inv_J * x_vals

            self.theta_1_vals += q_dot[0] * self.timer_period
            self.theta_2_vals += q_dot[1] * self.timer_period
            self.theta_3_vals += q_dot[2] * self.timer_period
            self.theta_4_vals += q_dot[3] * self.timer_period
            self.theta_5_vals += q_dot[4] * self.timer_period
            self.theta_6_vals += q_dot[5] * self.timer_period

            j1_theta = float(self.theta_1_vals)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)

            #self.j_angle = Float64MultiArray()
            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

            self.end_pos = self.A_6_wrt_0.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})
            
            if self.end_pos[1,3] <= 0.3:
                self.counter += 1
                self.r1 = ((self.end_pos[0,3]) ** 2) + ((self.end_pos[1,3]) ** 2) ** 0.5
                
                if self.cup_val is False:
                    self.get_logger().info('Table is clear, making coffee.')

                if self.cup_val is True:
                    self.get_logger().info('Remove cup from table and try again. Table is not clear.')
                    rclpy.shutdown()

                time.sleep(3)

                # Gripper Open
                gripper_msg = Float64MultiArray()
                gripper_msg.data = [-1.0, 1.0, 0.0, 0.0]  
                self.gripper_position_pub.publish(gripper_msg)
                




            self.joint_position_pub.publish(self.j_angle)


        if self.counter == 2: # Arc Path


            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()

            x_vals = Matrix([0.0, 0.0, 0.0 , 0.0, 0.0, 0.0]) 

            arc_time = 3.0
            
            #self.get_logger().info(f'arc_counter: {self.arc1_counter}')

            target_position = ((self.arc1_counter) / arc_time) * self.target_pose3
            #self.get_logger().info(f'target position {target_position}')


            #self.get_logger().info(f'theta 1 vals: {self.theta_1_vals}')

            j1_theta = float(self.theta_1_vals + target_position)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)

            #j_angle = Float64MultiArray()
            
            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]
            #self.get_logger().info(f'counter: {self.counter}')
        
            self.joint_position_pub.publish(self.j_angle)
            #self.get_logger().info(f'Published positions: {j_angle.data}')

            
            if self.arc1_counter >= arc_time:
                self.counter += 1
                self.end_pos = self.A_6_wrt_0.subs({theta_1: j1_theta, theta_2: j2_theta, theta_3: j3_theta, theta_4: j4_theta, theta_5: j5_theta, theta_6: j6_theta})
                self.theta_1_vals = j1_theta
                self.arc1_counter = 0.0

            self.arc1_counter += self.timer_period


        if self.counter == 3:

            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()
            
            x_vals = Matrix([-0.50, 0, 0.0 , 0.0, 0.0, 0.0])   

            q_dot = inv_J * x_vals

            self.theta_1_vals += q_dot[0] * self.timer_period
            self.theta_2_vals += q_dot[1] * self.timer_period
            self.theta_3_vals += q_dot[2] * self.timer_period
            self.theta_4_vals += q_dot[3] * self.timer_period
            self.theta_5_vals += q_dot[4] * self.timer_period
            self.theta_6_vals += q_dot[5] * self.timer_period

            j1_theta = float(self.theta_1_vals)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)

            #self.j_angle = Float64MultiArray()
            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

            self.end_pos = self.A_6_wrt_0.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})
            
            if self.end_pos[0,3] <= -0.88:

                # Gripper Close
                gripper_msg = Float64MultiArray()
                gripper_msg.data = [-0.25, 0.25,0.0,0.0]  
                self.gripper_position_pub.publish(gripper_msg)
                
                self.counter += 1
                self.r1 = ((self.end_pos[0,3]) ** 2) + ((self.end_pos[1,3]) ** 2) ** 0.5
                time.sleep(3)

            self.joint_position_pub.publish(self.j_angle)

   
        if self.counter == 4:

            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()
            
            x_vals = Matrix([0.50, 0, 0.0 , 0.0, 0.0, 0.0])   

            q_dot = inv_J * x_vals

            self.theta_1_vals += q_dot[0] * self.timer_period
            self.theta_2_vals += q_dot[1] * self.timer_period
            self.theta_3_vals += q_dot[2] * self.timer_period
            self.theta_4_vals += q_dot[3] * self.timer_period
            self.theta_5_vals += q_dot[4] * self.timer_period
            self.theta_6_vals += q_dot[5] * self.timer_period

            j1_theta = float(self.theta_1_vals)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)

            #self.j_angle = Float64MultiArray()
            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

            self.end_pos = self.A_6_wrt_0.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})
            
            if self.end_pos[0,3] >= -0.3:
                self.counter += 1
                self.r1 = ((self.end_pos[0,3]) ** 2) + ((self.end_pos[1,3]) ** 2) ** 0.5
            

            self.joint_position_pub.publish(self.j_angle)



        if self.counter == 5: # Arc Path


            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()

            x_vals = Matrix([0.0, 0.0, 0.0 , 0.0, 0.0, 0.0]) 

            arc_time = 3.0
            
      

            target_position = ((self.arc1_counter) / arc_time) * self.target_pose3


            j1_theta = float(self.theta_1_vals + target_position)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)
            

            if j1_theta >= math.pi:
                self.theta_1_vals = -math.pi
            #j_angle = Float64MultiArray()
            
            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]
            #self.get_logger().info(f'counter: {self.counter}')
        
            self.joint_position_pub.publish(self.j_angle)
            #self.get_logger().info(f'Published positions: {j_angle.data}')

            
            if self.arc1_counter >= arc_time:
                self.counter += 1
                self.end_pos = self.A_6_wrt_0.subs({theta_1: j1_theta, theta_2: j2_theta, theta_3: j3_theta, theta_4: j4_theta, theta_5: j5_theta, theta_6: j6_theta})
                self.theta_1_vals = j1_theta
                self.arc1_counter = 0.0


            self.arc1_counter += self.timer_period


        if self.counter == 6:

            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()
            
            x_vals = Matrix([0.0, -0.50, 0.0 , 0.0, 0.0, 0.0])   

            q_dot = inv_J * x_vals

            self.theta_1_vals += q_dot[0] * self.timer_period
            self.theta_2_vals += q_dot[1] * self.timer_period
            self.theta_3_vals += q_dot[2] * self.timer_period
            self.theta_4_vals += q_dot[3] * self.timer_period
            self.theta_5_vals += q_dot[4] * self.timer_period
            self.theta_6_vals += q_dot[5] * self.timer_period

            j1_theta = float(self.theta_1_vals)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)

            #self.j_angle = Float64MultiArray()
            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

            self.end_pos = self.A_6_wrt_0.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})
            
            if self.end_pos[1,3] <= -0.7:
                self.counter += 1
                self.r1 = ((self.end_pos[0,3]) ** 2) + ((self.end_pos[1,3]) ** 2) ** 0.5
                time.sleep(3)
            

            self.joint_position_pub.publish(self.j_angle)

   
        if self.counter == 7:

            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()
            
            x_vals = Matrix([0.0, 0.50, 0.0 , 0.0, 0.0, 0.0])   

            q_dot = inv_J * x_vals

            self.theta_1_vals += q_dot[0] * self.timer_period
            self.theta_2_vals += q_dot[1] * self.timer_period
            self.theta_3_vals += q_dot[2] * self.timer_period
            self.theta_4_vals += q_dot[3] * self.timer_period
            self.theta_5_vals += q_dot[4] * self.timer_period
            self.theta_6_vals += q_dot[5] * self.timer_period

            j1_theta = float(self.theta_1_vals)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)

            #self.j_angle = Float64MultiArray()
            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

            self.end_pos = self.A_6_wrt_0.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})
            
            if self.end_pos[1,3] >= -0.3:
                self.counter += 1
                self.r1 = ((self.end_pos[0,3]) ** 2) + ((self.end_pos[1,3]) ** 2) ** 0.5
            

            self.joint_position_pub.publish(self.j_angle)



        if self.counter == 8: # Arc Path


            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()

            x_vals = Matrix([0.0, 0.0, 0.0 , 0.0, 0.0, 0.0]) 

            arc_time = 3.0
            
            #self.get_logger().info(f'arc_counter: {self.arc1_counter}')           


            target_position = ((self.arc1_counter) / arc_time) * self.target_pose3
            #self.get_logger().info(f'target position {target_position}')


            #self.get_logger().info(f'theta 1 vals: {self.theta_1_vals}')

            j1_theta = float(self.theta_1_vals + target_position)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)
            

            if j1_theta >= math.pi:
                self.theta_1_vals = -math.pi
            #j_angle = Float64MultiArray()
            
            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]
            #self.get_logger().info(f'counter: {self.counter}')
        
            self.joint_position_pub.publish(self.j_angle)


            
            if self.arc1_counter >= arc_time:
                self.counter += 1
                self.end_pos = self.A_6_wrt_0.subs({theta_1: j1_theta, theta_2: j2_theta, theta_3: j3_theta, theta_4: j4_theta, theta_5: j5_theta, theta_6: j6_theta})
                self.theta_1_vals = j1_theta
                self.arc1_counter = 0.0


            self.arc1_counter += self.timer_period


        if self.counter == 9:

            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()
            
            x_vals = Matrix([0.50, 0, 0.0 , 0.0, 0.0, 0.0])   

            q_dot = inv_J * x_vals

            self.theta_1_vals += q_dot[0] * self.timer_period
            self.theta_2_vals += q_dot[1] * self.timer_period
            self.theta_3_vals += q_dot[2] * self.timer_period
            self.theta_4_vals += q_dot[3] * self.timer_period
            self.theta_5_vals += q_dot[4] * self.timer_period
            self.theta_6_vals += q_dot[5] * self.timer_period

            j1_theta = float(self.theta_1_vals)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)

   
            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

            self.end_pos = self.A_6_wrt_0.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})
            
            if self.end_pos[0,3] >= 0.7:
                self.counter += 1
                self.r1 = ((self.end_pos[0,3]) ** 2) + ((self.end_pos[1,3]) ** 2) ** 0.5
                time.sleep(3)

            self.joint_position_pub.publish(self.j_angle)

   
        if self.counter == 10:

            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()
            
            x_vals = Matrix([-0.50, 0, 0.0 , 0.0, 0.0, 0.0])   

            q_dot = inv_J * x_vals

            self.theta_1_vals += q_dot[0] * self.timer_period
            self.theta_2_vals += q_dot[1] * self.timer_period
            self.theta_3_vals += q_dot[2] * self.timer_period
            self.theta_4_vals += q_dot[3] * self.timer_period
            self.theta_5_vals += q_dot[4] * self.timer_period
            self.theta_6_vals += q_dot[5] * self.timer_period

            j1_theta = float(self.theta_1_vals)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)

            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

            self.end_pos = self.A_6_wrt_0.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})
            
            if self.end_pos[0,3] <= 0.3:
                self.counter += 1
                self.r1 = ((self.end_pos[0,3]) ** 2) + ((self.end_pos[1,3]) ** 2) ** 0.5
            
            self.joint_position_pub.publish(self.j_angle)



        if self.counter == 11: # Arc Path


            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()

            x_vals = Matrix([0.0, 0.0, 0.0 , 0.0, 0.0, 0.0]) 

            arc_time = 3.0


            target_position = ((self.arc1_counter) / arc_time) * self.target_pose3


            j1_theta = float(self.theta_1_vals + target_position)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)
            

            if j1_theta >= math.pi:
                self.theta_1_vals = -math.pi
            
            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

        
            self.joint_position_pub.publish(self.j_angle)


            
            if self.arc1_counter >= arc_time:
                self.counter += 1
                self.end_pos = self.A_6_wrt_0.subs({theta_1: j1_theta, theta_2: j2_theta, theta_3: j3_theta, theta_4: j4_theta, theta_5: j5_theta, theta_6: j6_theta})
                self.theta_1_vals = j1_theta
                self.arc1_counter = 0.0

            self.arc1_counter += self.timer_period



        if self.counter == 12:

            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()
            
            x_vals = Matrix([0.0, 0.50, 0.0 , 0.0, 0.0, 0.0])   

            q_dot = inv_J * x_vals

            self.theta_1_vals += q_dot[0] * self.timer_period
            self.theta_2_vals += q_dot[1] * self.timer_period
            self.theta_3_vals += q_dot[2] * self.timer_period
            self.theta_4_vals += q_dot[3] * self.timer_period
            self.theta_5_vals += q_dot[4] * self.timer_period
            self.theta_6_vals += q_dot[5] * self.timer_period

            j1_theta = float(self.theta_1_vals)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)

            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

            self.end_pos = self.A_6_wrt_0.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})
            
            if self.end_pos[1,3] >= 1.0:
                self.counter += 1
                self.r1 = ((self.end_pos[0,3]) ** 2) + ((self.end_pos[1,3]) ** 2) ** 0.5
                
                # Gripper Open
                gripper_msg = Float64MultiArray()
                gripper_msg.data = [-1.0, 1.0, 0.0, 0.0]  
                self.gripper_position_pub.publish(gripper_msg)
                
            self.joint_position_pub.publish(self.j_angle)

   
        if self.counter == 13:

            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()
            
            x_vals = Matrix([0.0, -0.50, 0.0 , 0.0, 0.0, 0.0])   

            q_dot = inv_J * x_vals

            self.theta_1_vals += q_dot[0] * self.timer_period
            self.theta_2_vals += q_dot[1] * self.timer_period
            self.theta_3_vals += q_dot[2] * self.timer_period
            self.theta_4_vals += q_dot[3] * self.timer_period
            self.theta_5_vals += q_dot[4] * self.timer_period
            self.theta_6_vals += q_dot[5] * self.timer_period

            j1_theta = float(self.theta_1_vals)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)

            self.j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

            self.end_pos = self.A_6_wrt_0.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})
            
            if self.end_pos[1,3] <= 0.3:
                self.counter += 1
                self.r1 = ((self.end_pos[0,3]) ** 2) + ((self.end_pos[1,3]) ** 2) ** 0.5

                #Gripper Close
                gripper_msg = Float64MultiArray()
                gripper_msg.data = [-0.25, 0.25,0.0,0.0]  
                self.gripper_position_pub.publish(gripper_msg)

                self.get_logger().info('Enjoy your coffee! Please restart the node to make another cup.')
            
            self.joint_position_pub.publish(self.j_angle)





def main(args=None):
    rclpy.init(args=args)
    coffee_node = CoffeePathNode()
    rclpy.spin(coffee_node)
    coffee_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
