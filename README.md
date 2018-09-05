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
**Goal is to calculate our DH Parameters table for the KUKA KR210 Robotic Arm:**

Steps:
- Create symbols for joint variables
- Sketch the manipulator in its zero configuration for identifiying each parameter
    - Label the joints from 1 to n
    - Draw line on the centerline of each joint axis (cylinder center)
    - Label each link from 0 (ground) to n
    - Define directions for the positive Z axes (collinear to each joint axis, and shifted to one common point for the gripper joints)
    - Define directions for the positive X axes (the X axis is defined by the common normals between Zi-1 and Zi)
    - The final sketch looks like this:
![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/KR210-DH-parameters-IMG1.png "Robot Arm DH-Parameter Sketch")

  
- Complete the DH parameter table in accordance to the manipulator configuration that was determined using the sketch:
    - Location (the "a's"):
        - a1 is the distance from Z1 to Z2 measured along X1 = 0.35
        - a2 is the distance from Z2 to Z3 measured along X2 = 1.25
        - a3 is the distance from Z3 to Z4 measured along X3
        - a4 (Z4 to Z5) = 0 (for collinear axes alpha and a are always 0)
        - a5 (Z5 to Z6) = 0
        - a6 (Z6 to ZG) = 0
    - Link offsets (the "d's"):
        - d1, the link offset, is the signed distance between from X0 to X1 meassured along Z1 (for collinear axes d1 is 0)
        - d2 = 0
        - d3 = 0
        - d4, the link offset or signed distance between from X3 to X4 meassured along Z4
        - d5 = 0
        - d6 = 0
        - dG =  the link offset or signed distance between from X6 to XG meassured along ZG
    - Twist angles (the alphas) between Zi and Zi+1:
        - α0, angle between Z0 and Z1 measured about x0 = 0
        - α1, angle between Z1 and Z2 measured about x1 = -90
        - α2, angle between Z2 and Z3 measured about x2 = 0
        - α3, angle between Z3 and Z4 measured about x3 = -90
        - α4, angle between Z4 and Z5 measured about x4 = 90
        - α5, angle between Z5 and Z6 measured about x5 = -90
        - α6, angle between Z6 and ZG measured about x6 = 0
    - We got so far a semi-populated DH-Parameter table:
![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/KR210-DH-parameters-table-so-far.jpg "DH-Parameter Table So Far Image")
        
    - Now get numerical values for the a's and d's from the **kr210.urdf.xacro** file
      (Script shortened for brevity):

	<!-- joints -->
	  <joint name="fixed_base_joint" type="fixed"> ...
	    <origin xyz="0 0 0" rpy="0 0 0"/>
	  </joint>
	  <joint name="joint_1" type="revolute"> ...
	    <origin xyz="0 0 0.33" rpy="0 0 0"/>
	    <axis xyz="0 0 1"/> ...
	  </joint>
	  <joint name="joint_2" type="revolute">
	    <origin xyz="0.35 0 0.42" rpy="0 0 0"/> ...
	    <axis xyz="0 1 0"/> ...
	  </joint>
	  <joint name="joint_3" type="revolute">
	    <origin xyz="0 0 1.25" rpy="0 0 0"/> ...
	    <axis xyz="0 1 0"/> ...
	  </joint>
	  <joint name="joint_4" type="revolute">
	    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/> ...
	    <axis xyz="1 0 0"/> ...
	  </joint>
	  <joint name="joint_5" type="revolute">
	    <origin xyz="0.54 0 0" rpy="0 0 0"/> ...
	    <axis xyz="0 1 0"/> ...
	  </joint>
	  <joint name="joint_6" type="revolute">
	    <origin xyz="0.193 0 0" rpy="0 0 0"/> ...
	    <axis xyz="1 0 0"/> ...
	  </joint>



    - Note that for joint3 we got an constant offset of -90° between X1 and X2. Due to this we map the "a" values from the "x" value in the URDF file until joint2 and then we map the "z" values to our "a" values. For the "d" values it is the other way around.
![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/KR210-reference-frame-URDF-to-DH-convension.jpg "Reference frame URDF to DH-convension")


    - In code we can now establish a dictionary of our known DH parameter quantities:

```
        s = {alpha0: 0, a0: 0, d1: 0.75,
             alpha1: -pi/2, a1: 0.35, d2: 0, q2: q2-pi/2,
             alpha2: 0, a2: 1.25, d3: 0,
             alpha3: -pi/2, a3: -0.054, d4: 1.5,
             alpha4: pi/2, a4: 0, d5: 0,
             alpha5: -pi/2, a5: 0, d6: 0,
             alpha6: 0, a6: 0, d7: 0.303, q7: 0}
```

### Second part of implementation
**Goal is to obtain individual transformation matrices from our calculated DH Parameters table:**


Steps (Please see **IK_server.py** for the specific implementation in Python):
- Create individual transformation matrices
- Substitute the DH table values into the expression with the subs method
- Create the transformaton matrix from the base frame to the end effector by composing the individual link transforms
- Initialize an empty list to be used as service response
- Start a loop to go through all the end-effector poses received from the request
- Get only trajectory points (positions) from the service message (it also contains velocities, accelerations, and efforts)
- Extract end-effector position (px,py,pz) and orientation (roll, pitch, yaw) from the request


### Third part of implementation
**Goal is to calculate the joint angles based on the position and orientation of the end-effector:**

Steps (Please see **IK_server.py** for the specific implementation in Python):
- Get end effector rotation matrix
- Create symbols for calculating the end effector rotation matrix
- Calculate each rotation matrix about each axis
- Obtain one single rotation matrix for the gripper by multiplying the yaw, pitch, and roll rotation matrices
- Compensate for rotation discrepancy between DH parameters and Gazebo
- Apply rotation error correction to align our DH parameters with that of the URDF file
- Create a matrix of the gripper position from the positions extracted from the end-effector poses received from the request
- Now calculate the wrist center using the end-effector POSITION (EE) and the end-effector ROTATION (ROT_EE)

Finally calculate joint angles (thetas) using the Geometric IK method:
- Calculate **theta1** using the wrist center
- Do a side-side-side triangle calculation for theta2 and theta3
- Calculate sides a, b and c
- Calculate correponding angles a, b and c
- Derive **theta2** and **theta3**
- Get the rotation matrix from base_link to link3 by multiplying the rotation matrices extracted from the transformation matrices
- Substitute the theta1,2,3 values into the rotation matrix from base_link to link3 using the subs method
- Calculate the rotation matrix from three to six. Take the rotation matrix of the end effector and multiply it by the inverse of the rotation matrix from base_link to link3
- As last step calculate **theta4**, **theta5** and **theta6**


### YouTube video
<a href="http://www.youtube.com/watch?feature=player_embedded&v=_KVFwSVJTrQ" target="_blank"><img src="http://img.youtube.com/vi/_KVFwSVJTrQ/0.jpg" 
alt="YouTube Video" width="240" height="180" border="10" /></a>

- Click on image above or visit this link: https://www.youtube.com/watch?v=_KVFwSVJTrQ
