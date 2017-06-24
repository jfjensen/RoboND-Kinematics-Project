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

            # T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
            #                [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
            #                [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
            #                [                   0,                   0,            0,               1]])
            # T0_1 = T0_1.subs(s)

            # T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
            #                [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
            #                [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
            #                [                   0,                   0,            0,               1]])
            # T1_2 = T1_2.subs(s)

            # T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
            #                [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
            #                [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
            #                [                   0,                   0,            0,               1]])
            # T2_3 = T2_3.subs(s)

            # T0_3 = simplify(T0_1 * T1_2 * T2_3)
            # R0_3 = T0_3[:3,:3]
            # print("R0_3: " + str(R0_3))
            #R0_3 = R0_3.subs(s)

            R0_3 = Matrix([[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
                           [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
                           [        cos(q2 + q3),        -sin(q2 + q3),        0]])

            
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
            # print(target_matrix[0,2])
            # print(target_matrix[1,2])
            # print(target_matrix[2,2])
            R0_6 = Matrix(tf.transformations.quaternion_matrix((req.poses[x].orientation.x, 
                                                        req.poses[x].orientation.y, 
                                                        req.poses[x].orientation.z, 
                                                        req.poses[x].orientation.w)))
            # print(R0_6)
            # print(R_corr)
            R0_6_corr = R_corr.T * R0_6 #* R_corr.T
            
            # R0_6_corr = R0_6 * R_corr**-1

            ### theta 1 ###########################################################################
            theta1 = atan2(py,px)
  
            WC = Matrix([[px - (s[d7]*R0_6_corr[0,2])],
                         [py - (s[d7]*R0_6_corr[1,2])],
                         [pz - (s[d7]*R0_6_corr[2,2])]])

  
            T0_2 = Matrix([[s[a1]*cos(theta1)],
                           [s[a1]*sin(theta1)],
                           [s[d1]]])

            g = WC - T0_2

      
            g_norm = sqrt(g[0]**2 + g[1]**2 + g[2]**2)

            l = sqrt((s[d4]**2) + (s[a3]**2))
      
        
            ### theta 3 ###########################################################################
            
            theta3_D = ((l**2) + (s[a2]**2) - (g_norm**2))/(2*l*s[a2])
                        
            theta3_phi = atan2(sqrt(1 - theta3_D**2),theta3_D)

            theta3 = -theta3_phi + atan2(s[d4], s[a3])
            
      
            ### theta 2 ###########################################################################
            
            theta2_D = ((g_norm**2) + (s[a2]**2) - (l**2))/(2*g_norm*s[a2])
            
            theta2_phi = atan2(sqrt(1 - theta2_D**2),theta2_D)

            theta2_beta = atan2(g[2], sqrt((g[0]**2)+(g[1]**2)))
            
            theta2 =  (pi/2) - theta2_phi - theta2_beta


            ### theta 4-5-6 #######################################################################

            R3_6 = R0_3.T * R0_6[:3,:3]
            R3_6 = R3_6.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            
            theta4 = atan2(R3_6[2,1], R3_6[2,2])
            theta5 = atan2(-R3_6[2,0], sqrt(R3_6[0,0]**2 + R3_6[1,0]**2))
            theta6 = atan2(R3_6[1,0], R3_6[0,0])
                        

            print("theta1: " + str(theta1) + " theta2: " + str(theta2) + " theta3: " + str(theta3))
            print("theta4: " + str(theta4) + " theta5: " + str(theta5) + " theta6: " + str(theta6))
            # print("theta4_b: " + str(theta4_b) + " theta5_b: " + str(theta5_b) + " theta6_b: " + str(theta6_b))

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
