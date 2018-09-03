from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 



    ## Insert IK code here! Starting at "Define DH parameters symbols" and ending at populate response for the IK request
    
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

    # Extract rotation matrices from the transformation matrices
    # In Sympy we are able to slice submatrices 
    R0_1 = T0_1[0:3,0:3]
    R1_2 = T1_2[0:3,0:3]
    R2_3 = T2_3[0:3,0:3]
    R3_4 = T3_4[0:3,0:3]
    R4_5 = T4_5[0:3,0:3]
    R5_6 = T5_6[0:3,0:3]
    R6_G = T6_G[0:3,0:3]
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
    ROT_EE = ROT_z * ROZ_y * ROT_x
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
    R3_6 = R0_3.inv("LU") * ROT_EE
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

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    #Calculate the complete transform from 0 to EE and substitute-in the found thetas:
    FK = T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [1,1,1] # <--- Load your calculated WC values in this array
    your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
