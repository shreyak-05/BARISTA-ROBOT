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

        self.J = Matrix([[q_1_x_dot, q_2_x_dot, q_3_x_dot, q_4_x_dot, q_5_x_dot, q_6_x_dot],
                    [q_1_y_dot, q_2_y_dot, q_3_y_dot, q_4_y_dot, q_5_y_dot, q_6_y_dot],
                    [q_1_z_dot, q_2_z_dot, q_3_z_dot, q_4_z_dot, q_5_z_dot, q_6_z_dot],
                    [Z_1, Z_2, Z_3, Z_4, Z_5, Z_6]])

        self.get_logger().info("Computed J")

 

 

        

        self.joint_1_position = None
        self.joint_2_position = None
        self.joint_3_position = None
        self.joint_4_position = None
        self.joint_5_position = None
        self.joint_6_position = None
        #self.lf1_position = None
        #self.lf2_position = None
        #self.rf1_position = None
        #self.rf2_position = None

        self.J_sub = None

        theta_1_vals = [0]
        theta_2_vals = [0.1]
        theta_3_vals = [-.383]
        theta_4_vals = [0.283]
        theta_5_vals = [0.0001]
        theta_6_vals = [0]

        
        #self.get_logger().info(f"6 wrt 0 Transformation Matrix:\n{J_sub}")
    
        # Creating Publishers
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        # Creating Subscriber
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)


        #self.publisher_ = self.create_publisher(Float64MultiArray, '/position_controller_1/commands', 10)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()

        self.duration1 = 5  # Total time to move from 0 to pi
        self.duration2 = 5
        self.duration3 = 5
        self.duration4 = 5

        self.total_duration = 20
        self.target_pose1 = math.pi/2
        self.target_pose2 = math.pi
        self.target_pose3 = (3*math.pi)/4



        self.get_logger().info("Joint Mover Node has started.")
        self.counter = 0
        self.arc_counter = 0.0
    
        self.theta_1_vals = 0.0
        self.theta_2_vals = 0.0
        self.theta_3_vals = 0.0
        self.theta_4_vals = 0.0
        self.theta_5_vals = 0.0
        self.theta_6_vals = 0.0

        self.target_position1 = 1.57
        self.target_position2 = math.pi/4
        self.target_position3 = math.pi/2
        self.target_position4 = math.pi/4
        self.target_position5 = 1.57
        self.target_position6 = -3.141592653589793


         if self.counter == 8: # Arc Path


            theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')

            J_sub = self.J.subs({theta_1: self.theta_1_vals, theta_2: self.theta_2_vals, theta_3: self.theta_3_vals, theta_4: self.theta_4_vals, theta_5: self.theta_5_vals, theta_6: self.theta_6_vals})

            inv_J = J_sub.pinv()

            x_vals = Matrix([0.0, 0.0, 0.0 , 0.0, 0.0, 0.0]) 

            arc_time = 3.0
            
            self.get_logger().info(f'arc_counter: {self.arc1_counter}')

            target_position = ((self.arc1_counter) / arc_time) * self.target_pose3
            self.get_logger().info(f'target position {target_position}')


            self.get_logger().info(f'theta 1 vals: {self.theta_1_vals}')

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


            self.get_logger().info(f'counter: {self.counter}')
            #self.joint_position_pub.publish(self.j_angle)
            self.get_logger().info(f'End Effector Position: {self.end_pos[0,3], self.end_pos[1, 3], self.end_pos[2, 3]}')
            self.get_logger().info(f'Published positions: {self.j_angle.data}')

            self.arc1_counter += self.timer_period



    def timer_callback(self):




        if self.counter == 0:

            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed_time >= self.duration1:
                #elapsed_time = self.duration1
                self.counter += 1

            arc_time = 3.0
            
            self.get_logger().info(f'arc_counter: {self.arc_counter}')

            target_position3 = ((self.arc_counter) / arc_time) * self.target_pose3
            self.get_logger().info(f'target position {target_position}')


            self.get_logger().info(f'theta 1 vals: {self.theta_1_vals}')

            j1_theta = float(self.theta_1_vals + target_position1)
            j2_theta = float(self.theta_2_vals)
            j3_theta = float(self.theta_3_vals + target_position3)
            j4_theta = float(self.theta_4_vals)
            j5_theta = float(self.theta_5_vals)
            j6_theta = float(self.theta_6_vals)
            
            j_angle = Float64MultiArray()
            j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]
            #self.get_logger().info(f'counter: {self.counter}')


            self.joint_position_pub.publish(j_angle)
            #self.get_logger().info(f'Published positions: {j_angle.data}')

        elif self.counter == 1:
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

            if elapsed_time >= self.duration1 + self.duration2:
                #elapsed_time = self.duration1
                self.counter += 1

            target_position = ((elapsed_time - self.duration1) / self.duration2) * self.target_pose2

            j1_theta = 1.57
            j2_theta = 0.0
            j3_theta = 0.0
            j4_theta = -target_position
            j5_theta = 1.57
            j6_theta = target_position

            j_angle = Float64MultiArray()
            j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

            #self.get_logger().info(f'counter: {self.counter}')
            self.joint_position_pub.publish(j_angle)
            #self.get_logger().info(f'Published positions: {j_angle.data}')


            self.arc_counter

        
        elif self.counter == 2:

            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9


            target_position = ((elapsed_time - self.duration1 - self.duration2) / self.duration3) * self.target_pose3


            
            j1_theta = 1.57
            j2_theta = -target_position/2
            j3_theta = target_position
            j4_theta = math.pi -target_position/2
            j5_theta = 1.57
            j6_theta = math.pi

            j_angle = Float64MultiArray()
            
            j_angle = Float64MultiArray()
            j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]
            #self.get_logger().info(f'counter: {self.counter}')

            if elapsed_time >= self.duration1 + self.duration2 + self.duration3:
                elapsed_time = self.duration1   
                self.get_logger().info("Motion complete. Stopping timer.")
                self.get_logger().info(f"Joint Positions: {j_angle.data}")
                self.timer.cancel()
        
            self.joint_position_pub.publish(j_angle)
            #self.get_logger().info(f'Published positions: {j_angle.data}')


    def joint_state_callback(self, msg):
        """Callback to process received joint state messages."""
        theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')



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




        self.joint_1_position = msg.position[index_1]
        self.joint_2_position = msg.position[index_2]
        self.joint_3_position = msg.position[index_3]
        self.joint_4_position = msg.position[index_4]
        self.joint_5_position = msg.position[index_5]
        self.joint_6_position = msg.position[index_6]

        self.J_sub = self.J.subs({theta_1: self.joint_1_position, theta_2: self.joint_2_position, theta_3: self.joint_3_position, theta_4: self.joint_4_position, theta_5: self.joint_5_position, theta_6: self.joint_6_position})
        
        
        #float_J_sub = 

        #t_vals = self.T.subs({theta_1: joint_1_position, theta_2: joint_2_position, theta_3: joint_3_position, theta_4: joint_4_position, theta_5: joint_5_position, theta_6: joint_6_position})
        #float_t_vals = list(map(float,t_vals))


        #j1_vel = 0.006

        #j1_theta = joint_1_position + (j1_vel * 0.01)
        #j2_theta = 0.0
        #j3_theta = 0.0
        #j4_theta = 0.0
        #j5_theta = 0.0
        #j6_theta = 0.0

        #j_angle = Float64MultiArray()
        #j_angle.data = [j1_theta, j2_theta, j3_theta, j4_theta, j5_theta, j6_theta]

        #self.joint_position_pub.publish(j_angle)
        #self.get_logger().info('publish angle')

        #if joint_1_position < 1.57:
        #    self.joint_velocity_pub.publish(j_vel)
        #    self.get_logger().info('moving j1')
        #if joint_1_position >= 1.57:
        #    self.joint_velocity_pub.publish(j_stop)
        #    self.get_logger().info('Reached the goal! Stopping the robot.')



def main(args=None):
    rclpy.init(args=args)
    velocity_node = JointVelocityNode()
    rclpy.spin(velocity_node)
    velocity_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()