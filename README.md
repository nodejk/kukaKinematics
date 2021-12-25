# kukaKinematics
Implementation of Forwards Kinematics, Inverse Kinematics, Point to Point Movement and Synchronous movement for Kuka KR 120 R2700-2.

The DH table for the robot is given as follows:

    n    theta       alpha     a      d
    0      0         -180      0     645
    1    theta1       90      330     0
    2    theta2        0      1150    0
    3   theta3-90     90      115     0
    4    theta4      -90       0    -1220
    5    theta5       90       0      0
    6  theta6-180   -180       0    -215

Data Sheet:
https://www.kuka.com/-/media/kuka-downloads/imported/6b77eecacfe542d3b736af377562ecaa/0000325899_en.pdf?rev=fcdf2d2c871e4a10a85f45c68446c4fc&hash=051C01329D7D3501F592EC3B1AB99864

kuka.py has a class named kuka. It initializes with all the kuka dh parameters (a, alpha and d).

forwardKinematics: This method outputs the final position and orientation of the robot for a given configuration.
    
    req: theta1, theta2, theta3, theta4, theta5 and theta6.
    output: 4X4 end transformation matrix.
 
 
 inverseKinematics: This method outputs all the configurations in a form of list using geometry. Note that angles are according 
 to the angles direction provided in the datasheet. This method does not take into consideration the physical contraints of the robot.
 At max, there are 48 configurations which can be present for the robot. The breakdown is as follows:
 
   1. 3 configurations for the first joint.
   2. 2 configurations for elbow up and elbow down.
   3. 2 configurations for wrist up and wrist down orientation.
   4. 4 configurations for the joint 4 and 6.

    req: 4X4 end transformation matrix.
    output: [[theta1, theta2,...theta6], [theta1, theta2,...theta6]...]
    
line2lineMovement: This method returns a list of configurations required to reach from point A to point B in a straight line. It assumes the 
starting and the end oritentation to be same. To find if the movement is even possible it finds the optimal orientation according to the plausible speed of each joint and physical contraints of them provied in the datasheet above. timeSample is the sampling time for the given robot.

    req: startTransformationMatrix, endTransformationMatrix, velocity, acceleration, timeSample.
    output: [[theta1, theta2,...theta6], [theta1, theta2,...theta6]...]


