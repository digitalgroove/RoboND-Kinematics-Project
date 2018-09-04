#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import * # we import the service messages from kuka_arm package
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # used to pack individual joint variables into a single message
from geometry_msgs.msg import Pose # in test mode, we will receive an IK request from the simulator with end-effector poses
from mpmath import * # or facilitating symbolic math in python
from sympy import *
import numpy as np
from numpy import array


def handle_calculate_IK(req):
    # callback function for when a request for the CalculateIK service type is received
    # print out the number of end-effector poses received from the request
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### First part of implementation is to obtain individual transformation matrices from our calculated DH Parameters table:
        # Create symbols for joint variables
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	#
	# Create Modified DH parameters
	# KUKA KR210
        s = {alpha0: 0, a0: 0, d1: 0.75,
             alpha1: -pi/2, a1: 0.35, d2: 0, q2: q2-pi/2,
             alpha2: 0, a2: 1.25, d3: 0,
             alpha3: -pi/2, a3: -0.054, d4: 1.5,
             alpha4: pi/2, a4: 0, d5: 0,
             alpha5: -pi/2, a5: 0, d6: 0,
             alpha6: 0, a6: 0, d7: 0.303, q7: 0}
	#
	# Define Modified DH Transformation matrix
	# Create individual transformation matrices
        T0_1 = Matrix([[            cos(q1),            -sin(q1),            0,              a0],
                       [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                  0,                   0,            0,               1]])

        T1_2 = Matrix([[           cos(q2),            -sin(q2),            0,              a1],
                      [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                      [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                      [                  0,                   0,            0,               1]])


        T2_3 = Matrix([[           cos(q3),            -sin(q3),            0,              a2],
                      [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                      [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                      [                  0,                   0,            0,               1]])

        T3_4 = Matrix([[           cos(q4),            -sin(q4),            0,              a3],
                      [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                      [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                      [                  0,                   0,            0,               1]])

        T4_5 = Matrix([[           cos(q5),            -sin(q5),            0,              a4],
                      [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                      [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                      [                  0,                   0,            0,               1]])

        T5_6 = Matrix([[           cos(q6),            -sin(q6),            0,              a5],
                      [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                      [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                      [                  0,                   0,            0,               1]])

        T6_G = Matrix([[           cos(q7),            -sin(q7),            0,              a6],
                      [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                      [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                      [                  0,                   0,            0,               1]])
        
        # Substitute the DH table values into the expression with the subs method
        T0_1 = T0_1.subs(s)
        T1_2 = T1_2.subs(s)
        T2_3 = T2_3.subs(s)
        T3_4 = T3_4.subs(s)
        T4_5 = T4_5.subs(s)
        T5_6 = T5_6.subs(s)
        T6_G = T6_G.subs(s)

        # Create the transformation matrix from the base frame to the end effector by composing the individual link transforms
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
	#
        ######

        # Initialize an empty list to be used as service response
        joint_trajectory_list = [] # it will contain the joint angle values that we calculate here
        for x in xrange(0, len(req.poses)): # start a loop to go through all the end-effector poses received from the request
            # IK code starts here
            # Contains joint angle positions, velocities, accelerations, and efforts. 
            # We will only use position field for a specific end-effector position
            joint_trajectory_point = JointTrajectoryPoint() 


	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Second part of implementation is to calculate the joint angles based on the position and orientation of the end-effector
	    # We start by getting end effector rotation matrix 
            #
            # Create symbols for calculating the end effector rotation matrix
            r, p, y = symbols('r p y')
            #
            # Calculate each rotation matrix about each axis
            ROT_x = Matrix([[1,      0,       0],
                            [0, cos(r), -sin(r)],
                            [0, sin(r),  cos(r)]]) # ROLL

            ROT_y = Matrix([[ cos(p),      0,  sin(p)],
                            [      0,      1,       0],
                            [-sin(p),      0,  cos(p)]]) # PITCH

            ROT_z = Matrix([[cos(y), -sin(y), 0],
                            [sin(y),  cos(y), 0],
                            [     0,       0, 1]]) # YAW
            # 
            # Obtain one single rotation matrix for the gripper by multiplying the yaw, pitch, and roll rotation matrices
            ROT_EE = ROT_z * ROT_y * ROT_x
            #
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            # Apply rotation error correction to align our DH parameters with that of the URDF file
            Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
            ROT_EE = ROT_EE * Rot_Error
            #
            # Create a matrix of the gripper position from the positions extracted from the end-effector poses received from the request
            ROT_EE = ROT_EE.subs({'r':roll, 'p':pitch, 'y':yaw})
            EE = Matrix([[px],
                        [py],
                        [pz]])
            # 
            # We can now calculate the wrist center using the end-effector POSITION (EE) and the end-effector ROTATION (ROT_EE)
            WC = EE - (0.303) * ROT_EE[:,2]
	    #
	    #
	    # Finally calculate joint angles (thetas) using the Geometric IK method
	    # Calculate theta1 using the wrist center
            theta1 = atan2(WC[1],WC[0])
            #
            # Side-side-side triangle calculation for theta2 and theta3
            # Calculate sides a, b and c
            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            side_c = 1.25
            # 
	    # Calculate correponding angles a, b and c
            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))
            #
            # Derive theta2 and theta3
            theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC [1]) - 0.35)
            theta3 = pi / 2 - (angle_b + 0.036)  # 0.036 accounts for sag in link4 of -0.54m
            #
            # Get the rotation matrix from base_link to link3 by multiplying the rotation matrices 
            # extracted from the transformation matrices
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            # 
            # Substitute the theta1,2,3 values into the rotation matrix from base_link to link3 using the subs method
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            # 
            # Now we calculate the rotation matrix from three to six. For that we take the rotation matrix of the end effector
            # and multiply it by the inverse of the rotation matrix from base_link to link3
            R3_6 = R0_3.transpose() * ROT_EE
            # 
            #
            # Our last step is to calculate theta4, theta5 and theta6
            # We calculate euler angles from rotation matrix
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            #
            # 
            ######

            # Populate response for the IK request
            # After calculating individual joint angles for a given eef pose,
            # we populate the joint_trajectory_list and send back as a response       
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize IK_server node and create (declare) a CalculateIK type service with the name calculate_ik
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
