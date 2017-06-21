#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []

        

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
            print("==============================================")
            theta1, theta2, theta3, theta4, theta5, theta6 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            # Define DH param symbols
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
                        
            # Joint angle symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

      
            # Modified DH params

            s = {alpha0:     0, a0:      0, d1:  0.75, 
                 alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,  
                 alpha2:     0, a2:   1.25, d3:     0,
                 alpha3: -pi/2, a3: -0.054, d4:  1.50,
                 alpha4:  pi/2, a4:      0, d5:     0,
                 alpha5: -pi/2, a5:      0, d6:     0,
                 alpha6:     0, a6:      0, d7: 0.303, q7: 0
                 }
            
            # Define Modified DH Transformation matrix
            R_z = Matrix([[ cos(pi),-sin(pi), 0, 0],
              [ sin(pi), cos(pi), 0, 0],
              [       0,       0, 1, 0],
              [       0,       0, 0, 1]])

            R_y = Matrix([[ cos(-pi/2), 0, sin(-pi/2), 0],
                                [          0, 1,          0, 0],
                          [-sin(-pi/2), 0, cos(-pi/2), 0],
                          [          0, 0,          0, 1]])

            R_corr = simplify(R_z * R_y)


            # Create individual transformation matrices


            
            # Extract end-effector position and orientation from request
    	    # px,py,pz = end-effector position
    	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            print("p: " + str(px) + " " + str(py) +  " " + str(pz))
     
            # Calculate joint angles using Geometric IK method
            target_matrix = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix((px,py,pz)),
                tf.transformations.quaternion_matrix((req.poses[x].orientation.x, req.poses[x].orientation.y, req.poses[x].orientation.z, req.poses[x].orientation.w)))
            # print(target_matrix)
            theta1 = atan2(py,px)

            P_2_4_K0 = Matrix([[px - (d6*target_matrix[0,2]) - a1*cos(theta1)],
                               [py - (d6*target_matrix[1,2]) - a1*sin(theta1)],
                               [pz - (d6*target_matrix[2,2]) - d1]])
            P_2_4_K0 = P_2_4_K0.subs(s)
            P_2_4_K0_norm = np.linalg.norm(P_2_4_K0)
            l1 = d4 ##################################################################
            phi_1 = asin(((l1*l1)-(a2*a2)+(P_2_4_K0_norm*P_2_4_K0_norm))/(2*P_2_4_K0_norm*l1))
            phi_1 = phi_1.subs(s)
            phi_2 = asin((P_2_4_K0_norm - (((l1*l1) - (a2*a2) + (P_2_4_K0_norm*P_2_4_K0_norm))/(2*P_2_4_K0_norm)))/a2)
            phi_2 = phi_2.subs(s)
            # theta3 = pi - (phi_1 + phi_2) - atan2(-d4, a3)
            theta3 = pi - (phi_1 + phi_2) + atan2(-d4, a3)            
            theta3 = theta3.subs(s)

            target_matrix_R = target_matrix[:3,:3]
            P_2_4_K2 = (target_matrix_R.T)*P_2_4_K0
            beta1 = atan2(P_2_4_K2[0],P_2_4_K2[1])
            beta2_1 = asin(((a2*a2) - (P_2_4_K0_norm*P_2_4_K0_norm) + (l1*l1))/(2*l1*a2))
            beta2_1 = beta2_1.subs(s)
            beta2_2 = asin((l1 - (((a2*a2) - (P_2_4_K0_norm*P_2_4_K0_norm) + (l1*l1))/(2*l1)))/P_2_4_K0_norm)
            beta2_2 = beta2_2.subs(s)
            # theta2 = (pi/2) - (abs(beta1) + (beta2_1 + beta2_2))
            # theta2 =  - (abs(beta1) + (beta2_1 + beta2_2))
            # theta2 =  - (pi/2) + (abs(beta1) - (beta2_1 + beta2_2))
            theta2 =  ((pi/2) - (beta1 + (beta2_1 + beta2_2)))*0.5
            
            theta2 = theta2.subs(s)


            print("theta1: " + str(theta1) + " theta2: " + str(theta2) + " theta3: " + str(theta3))
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
    	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
