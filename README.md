[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Robotic arm - Pick & Place project

Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.

### First part of implementation is to obtain individual transformation matrices from our calculated DH Parameters table:
- Create symbols for joint variables
- Define Modified DH Transformation matrix
- Create individual transformation matrices
- Substitute the DH table values into the expression with the subs method
- Create the transformaton matrix from the base frame to the end effector by composing the individual link transforms
- Extract rotation matrices from the transformation matrices, in Sympy we are able to slice submatrices
- Initialize an empty list to be used as service response
- IK code starts here:
- Contains joint angle positions, velocities, accelerations, and efforts. 
- We will only use position field for a specific end-effector position
- Extract end-effector position and orientation from request
- px,py,pz = end-effector position
- roll, pitch, yaw = end-effector orientation

### Second part of implementation is to calculate the joint angles based on the position and orientation of the end-effector
- We start by getting end effector rotation matrix
- Create symbols for calculating the end effector rotation matrix
- Calculate each rotation matrix about each axis
- Obtain one single rotation matrix for the gripper by multiplying the yaw, pitch, and roll rotation matrices
- Compensate for rotation discrepancy between DH parameters and Gazebo
- Apply rotation error correction to align our DH parameters with that of the URDF file
- Create a matrix of the gripper position from the positions extracted from the end-effector poses received from the request
- We can now calculate the wrist center using the end-effector POSITION (EE) and the end-effector ROTATION (ROT_EE)
- Finally calculate joint angles (thetas) using the Geometric IK method
- Calculate theta1 usig the wrist center
- Side-side-side triangle calculation for theta2 and theta3
- Calculate sides a, b and c
- Calculate correponding angles a, b and c
- Derive theta2 and theta3
- Get the rotation matrix from base_link to link3 by multiplying the rotation matrices 
- extracted from the transformation matrices
- Substitute the theta1,2,3 values into the rotation matrix from base_link to link3 using the subs method
- Now we calculate the rotation matrix from three to six. For that we take the rotation matrix of the end effector
- and multiply it by the inverse of the rotation matrix from base_link to link3
- Our last step is to calculate theta4, theta5 and theta6
- We calculate euler angles from rotation matrix
