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
        s = {alpha0: 0, a0: 0, d1: 0.75
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
	#
	# Extract rotation matrices from the transformation matrices
	#
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
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
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
