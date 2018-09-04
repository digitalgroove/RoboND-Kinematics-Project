[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Robotic arm - Pick & Place project

I programmed the inverse kinematics for simulated KUKA KR210 robot arm. It grasps objects off a shelf and drops them into a container. This is one of the many action that robots have to manage at the Amazon Robotics Challenge.

![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/Pick_and_Place_Image.png "Robotic Arm Image")

Disclaimer: please refer to the original repository for the second project in the Udacity Robotics Nanodegree found here: https://github.com/udacity/RoboND-Kinematics-Project

### Dependencies:
You should have a Desktop-Full Install of ROS Kinetic and MoveIt!


### Running the project:

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

You will also need to run the Inverse Kinematics scrip separetely by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Note: the **demo** flag has to be set to _"false"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

This should be the default value, so there should be no need to make any changes when running the project for the first time. 


Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	- Shelf
	- Blue cylindrical target in one of the shelves
	- Dropbox right next to the robot
	

Once all these items are confirmed, open rviz window, hit Next or Continue button.



### First part of implementation 
**Goal is to obtain individual transformation matrices from our calculated DH Parameters table:**

Steps:
- Create symbols for joint variables
- Define Modified DH Transformation matrix
- Create individual transformation matrices
- Substitute the DH table values into the expression with the subs method
- Create the transformaton matrix from the base frame to the end effector by composing the individual link transforms
- Initialize an empty list to be used as service response
- Start a loop to go through all the end-effector poses received from the request
- Get only trajectory points (positions) from the service message (it also contains velocities, accelerations, and efforts)
- Extract end-effector position (px,py,pz) and orientation (roll, pitch, yaw) from the request


### Second part of implementation 
**Goal is to calculate the joint angles based on the position and orientation of the end-effector:**

Steps:
- Get end effector rotation matrix
- Create symbols for calculating the end effector rotation matrix
- Calculate each rotation matrix about each axis
- Obtain one single rotation matrix for the gripper by multiplying the yaw, pitch, and roll rotation matrices
- Compensate for rotation discrepancy between DH parameters and Gazebo
- Apply rotation error correction to align our DH parameters with that of the URDF file
- Create a matrix of the gripper position from the positions extracted from the end-effector poses received from the request
- Now calculate the wrist center using the end-effector POSITION (EE) and the end-effector ROTATION (ROT_EE)

Finally calculate joint angles (thetas) using the Geometric IK method:
- Calculate theta1 using the wrist center
- Do a side-side-side triangle calculation for theta2 and theta3
- Calculate sides a, b and c
- Calculate correponding angles a, b and c
- Derive theta2 and theta3
- Get the rotation matrix from base_link to link3 by multiplying the rotation matrices extracted from the transformation matrices
- Substitute the theta1,2,3 values into the rotation matrix from base_link to link3 using the subs method
- Calculate the rotation matrix from three to six. Take the rotation matrix of the end effector and multiply it by the inverse of the rotation matrix from base_link to link3
- As last step calculate theta4, theta5 and theta6


### YouTube video
<a href="http://www.youtube.com/watch?feature=player_embedded&v=_KVFwSVJTrQ" target="_blank"><img src="http://img.youtube.com/vi/_KVFwSVJTrQ/0.jpg" 
alt="YouTube Video" width="240" height="180" border="10" /></a>

- Click on image above or visit this link: https://www.youtube.com/watch?v=_KVFwSVJTrQ
