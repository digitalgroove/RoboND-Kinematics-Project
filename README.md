[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Robotic arm - Pick & Place project

I programmed the inverse kinematics for simulated KUKA KR210 robot arm. It grasps objects off a shelf and drops them into a container. This is one of the many action that robots have to manage at the Amazon Robotics Challenge.

![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/Pick_and_Place_Image.png "Robotic Arm Image")

**Screenshot of the completed pick and place process**

Disclaimer: please refer to the original repository for the second project in the Udacity Robotics Nanodegree found here: https://github.com/udacity/RoboND-Kinematics-Project

#### Table of Contents
1. Dependencies
2. First part of implementation: DH Parameters
3. Second part of implementation: Forward Kinematics
4. Third part of implementation: Inverse Kinematics
   1. Decouple Inverse Kinematics: calculate the Wrist Center
   2. Inverse Position Kinematics
   3. Inverse Orientation Kinematics
5. YouTube Video
6. Running the project

## 1. Dependencies
- You should have a Desktop-Full Install of ROS Kinetic and MoveIt!
- SymPy

## 2. First part of implementation: DH Parameters
**Goal is to calculate our DH Parameters table for the KUKA KR210 Robotic Arm.**

Steps:
- Create SymPy symbols for joint variables
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
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

## 3. Second part of implementation: Forward Kinematics
**Goal is to obtain the position of the end-effector by calculating the individual transformation matrices from our calculated DH Parameters table.**

Steps (please see **IK_server.py** for the full implementation in Python):
- Define the individual transformation matrices
  Recall the total transform matrix between adjacent coordinate frames:
  ![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/transform-matrix-between-adjacent-coordinate-frames.png "Transform matrix between adjacent coordinate frames")

  It is the best to first create transforms symbolically, then substitute numerical values for the non-zero terms as the last step.

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
- Now create the individual transformation matrices by substituting the DH table values into the expression with the SymPy subs method
```
        T0_1 = T0_1.subs(s)
        T1_2 = T1_2.subs(s)
        T2_3 = T2_3.subs(s)
        T3_4 = T3_4.subs(s)
        T4_5 = T4_5.subs(s)
        T5_6 = T5_6.subs(s)
        T6_G = T6_G.subs(s)
```
- Create the transformation matrix from the base frame to the end effector by composing the individual link transforms
  Foward Kinematics is the complete transform from 0 to EE:
```
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```
- Initialize an empty list to be used as service response
- Start a loop to go through all the end-effector poses received from the request
- Get only trajectory points (positions) from the service message (it also contains velocities, accelerations, and efforts)

## 4.Third part of implementation: Inverse Kinematics
**Goal is to calculate the joint angles based on the position and orientation of the end-effector.**

Steps (please see **IK_server.py** for the full implementation in Python):

- Extract one end-effector position (px,py,pz) and orientation (roll, pitch, yaw) from the request
```
px = req.poses[x].position.x
py = req.poses[x].position.y
pz = req.poses[x].position.z

(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
     [req.poses[x].orientation.x, req.poses[x].orientation.y,
       req.poses[x].orientation.z, req.poses[x].orientation.w])
```

### i.Decouple Inverse Kinematics: calculate the Wrist Center

In order to decouple the inverse kinematics into a **Inverse Position Kinematics** and **Inverse Orientation Kinematics** problem we need first to calculate the **Wrist Center**

Calculate the rotation of the end effector about its axes using the orientation information received from the request:

- Create SymPy symbols for calculating the end effector rotation matrix
```
r, p, y = symbols('r p y')
```

- Calculate each rotation matrix about each axis

  A **roll** is a counterclockwise rotation of gamma about the x-axis. The **roll** rotation matrix is given by:

  ![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/roll-rotation-matrix.gif "Rotation about the x-axis") 

```
            ROT_x = Matrix([[1,      0,       0],
                            [0, cos(r), -sin(r)],
                            [0, sin(r),  cos(r)]]) # ROLL
```
  A **pitch** is a counterclockwise rotation of beta about the y-axis. The **pitch** rotation matrix is given by:

  ![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/pitch-rotation-matrix.gif "Rotation about the y-axis") 


```
            ROT_y = Matrix([[ cos(p),      0,  sin(p)],
                            [      0,      1,       0],
                            [-sin(p),      0,  cos(p)]]) # PITCH
```
  A **yaw** is a counterclockwise rotation of α (alpha) about the z-axis. The **yaw** rotation matrix is given by:

  ![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/yaw-rotation-matrix.gif "Rotation about the z-axis")

```
            ROT_z = Matrix([[cos(y), -sin(y), 0],
                            [sin(y),  cos(y), 0],
                            [     0,       0, 1]]) # YAW
```

- Next obtain one single rotation matrix for the gripper by multiplying the yaw, pitch, and roll rotation matrices

  ![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/one-single-rotation-matrix.gif "Obtain one single rotation matrix")
``` 
ROT_EE = ROT_z * ROT_y * ROT_x
```

- Compensate for rotation discrepancy between DH parameters and Gazebo
  Apply rotation error correction to align our DH parameters with that of the URDF file
``` 
Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
ROT_EE = ROT_EE * Rot_Error
```

- Create a matrix of the gripper position from the positions extracted from the end-effector poses received from the request
            ROT_EE = ROT_EE.subs({'r':roll, 'p':pitch, 'y':yaw})
            EE = Matrix([[px],
                        [py],
                        [pz]])

- Now we can calculate the **wrist center** using the end-effector POSITION (EE) and the end-effector ROTATION (ROT_EE)
``` 
WC = EE - (0.303) * ROT_EE[:,2]
```

### ii.Inverse Position Kinematics 

Next we calculate the joint angles 1, 2 and 3 (thetas1, 2, 3) using the Geometric Inverse Kinematics method. We do not care about the orientation of the gripper at this point.

- Calculate **theta1** using the position of the wrist center (WC)
  Note that WC is a column vector that contains the cartesian coordinates x,y,z of the wrist center
  The orientation of joint1 is therefore the orientation of the wrist center
``` 
theta1 = atan2(WC[1],WC[0])  # atan2(WC_y, WC_x)
```
**Next calculate theta2 and theta3**

  Note that joint2 (theta2) and joint3 (theta3) have to accomodate so that the arms of the robot fit between the position of the wrist center and the position of joint2.
  ![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/Triangle_for_derivation_of_theta_2_and_theta_3.png "Triangle for derivation of theta2 and theta3")

  We have to do a side-side-side triangle calculation.

- Calculate sides a, b and c:
``` 
side_a = 1.501  # known distance between joint3 and the wrist center
side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
side_c = 1.25 # known distance between joint2 and joint3
```
  
- Calculate corresponding angles a, b and c:

``` 
angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))
```

- Derive **theta2**:
    ![alt text](https://github.com/digitalgroove/RoboND-Kinematics-Project/blob/master/misc_images/Derivation_of_theta_2.png "Derivation of theta2")

            theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC [1]) - 0.35)


- Similarly we derive **theta3**:
 
            theta3 = pi / 2 - (angle_b + 0.036)


### iii.Inverse Orientation Kinematics

- Get the rotation matrix from base_link to link3 by multiplying the rotation matrices extracted from the transformation matrices

            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]

- Substitute the real theta1,2,3 values into the SymPy rotation matrix from base_link to link3 using the subs method

            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

- Calculate the rotation matrix from three to six. Take the rotation matrix of the end effector and multiply it by the inverse of the rotation matrix from base_link to link3

            R3_6 = R0_3.transpose() * ROT_EE


- As last step calculate **theta4**, **theta5** and **theta6**
  
  We obtain euler angles for theta4, theta5 and theta6 from rotation matrix R3_6:
  
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

## 5.YouTube Video
<a href="http://www.youtube.com/watch?feature=player_embedded&v=_KVFwSVJTrQ" target="_blank"><img src="http://img.youtube.com/vi/_KVFwSVJTrQ/0.jpg" 
alt="YouTube Video" width="240" height="180" border="10" /></a>

- Click on image above or visit this link: https://www.youtube.com/watch?v=_KVFwSVJTrQ


## 6.Running the project

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
	

Once all these items are confirmed, open rviz window, hit Next (or Continue) button.



