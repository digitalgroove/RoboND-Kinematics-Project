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


def handle_calculate_IK(req):
    # callback function for when a request for the CalculateIK service type is received
    # print out the number of end-effector poses received from the request
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### First part of implementation is to obtain individual transformation matrices from our calculated DH Parameters table:
        # Create symbols
	#
	#
	# Create Modified DH parameters
	#
	#
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
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
