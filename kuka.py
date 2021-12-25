import numpy as np
import math

def dh_matrix(theta, alpha, r, d):
    """
    This function is used to form the transformation matrix required for the DH table.
    :param theta: angle of rotation around z(i).
    :param alpha: angle of rotation around x(i+1).
    :param r: displacement from i to i+1 frame along the x(i+1) axis.
    :param d: displacement from i to i+1 frame along the z(i) axis.
    :return: 4X4 transformation matrix containing the orientation and the position of the i+1 w.rt. i frame
                according to the given parameters.
    """
    theta_ = math.radians(theta)
    alpha_ = math.radians(alpha)

    a = np.matrix([ [math.cos(theta_), -1 * math.sin(theta_) * math.cos(alpha_), math.sin(theta_) * math.sin(alpha_), r * math.cos(theta_)],
                    [math.sin(theta_), math.cos(theta_) * math.cos(alpha_), -1 * math.cos(theta_) * math.sin(alpha_), r * math.sin(theta_)],
                    [0               , math.sin(alpha_)                   , math.cos(alpha_)                         , d],
                    [0               , 0                                  , 0                                        , 1]])

    return a

def magnitudeVector(x, y, z):
    """
    Returns the magnitude of a vector.
    :param x: x coordinate
    :param y: y coordinate
    :param z: z coordinate
    :return: magnitude(x, y, z)
    """
    return pow(x * x + y * y + z * z, 0.5)

def getHypotenuse(a, b):
    """
    Returns the length of hypotenuse of a right angle triangle having sides a and b.
    :param a: length of one side of the right triangle.
    :param b: length of another side of the right triangle
    :return: length of the hypotenuse.
    """
    return pow(a * a + b * b, 0.5)



class kuka:
    """
    This implementation is for the 6-DOF robot
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


    """
    def __init__(self):
        """
        This initializes the kuka with all the arms lengths according to the datasheet.
        """
        self.a0 = 0
        self.d0 = 645
        self.alpha0 = -180

        self.a1 = 330
        self.d1 = 0
        self.alpha1 = 90

        self.a2 = 1150
        self.d2 = 0
        self.alpha2 = 0

        self.a3 = 115
        self.d3 = 0
        self.alpha3 = 90

        self.a4 = 0
        self.d4 = -1220
        self.alpha4 = -90

        self.a5 = 0
        self.d5 = 0
        self.alpha5 = 90

        self.a6 = 0
        self.d6 = -215
        self.alpha6 = -180

    def forwardKinematics(self, theta1, theta2, theta3, theta4, theta5, theta6):
        """
        All the angles should be given in degrees.
        This method calculates final position and orientation of the end flange using
        the theta1, theta2, theta3, theta4, theta5, theta6.
        :param theta1: rotation around the 1st angle in degrees.
        :param theta2: rotation around the 2nd angle in degrees.
        :param theta3: rotation around the 3rd angle in degrees.
        :param theta4: rotation around the 4th angle in degrees.
        :param theta5: rotation around the 5th angle in degrees.
        :param theta6: rotation around the 6th angle in degrees.
        :return: np array of 4X4 transformation matrix.
        """
        a1 = dh_matrix(0, self.alpha0, self.a0, self.d0)
        a2 = dh_matrix(theta1, self.alpha1, self.a1, self.d1)
        a3 = dh_matrix(theta2, self.alpha2, self.a2, self.d2)
        a4 = dh_matrix(theta3 - 90, self.alpha3, self.a3, self.d3)
        a5 = dh_matrix(theta4, self.alpha4, self.a4, self.d4)
        a6 = dh_matrix(theta5, self.alpha5, self.a5, self.d5)
        a7 = dh_matrix(theta6 - 180, self.alpha6, self.a6, self.d6)
        a_final = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(a1, a2), a3), a4), a5), a6), a7)

        return a_final

    def getWristPosition(self, endEffector):
        """
        This function returns the position of the wrist transformation matrix for a given EndEffector.
        :param endEffector: 4X4 transformation matrix of the end effector.
        :return: coordinates of the end effector.
        """
        dg = abs(self.d6)
        nx, ny, nz = endEffector[0, 2], endEffector[1, 2], endEffector[2, 2]

        xWrist = endEffector[0, 3] - nx * dg
        yWrist = endEffector[1, 3] - ny * dg
        zWrist = endEffector[2, 3] - nz * dg

        return [xWrist, yWrist, zWrist]

    def inverseKinematics(self, endTransformationMatrix):
        """
        This function is used for the inverse kinematics.
        This calculates all the configurations for a given end effector position and orientation
        using geometry.
        Note that these might be all the valid possible configurations for the robot
        because of physical constraints.
        Additionally, this method returns configurations according to the direction of the robot
        defined in the datasheet.
        :param endTransformationMatrix: 4X4 end transformation matrix.
        :return: [[theta1, theta2,...theta6], [theta1, theta2,...theta6]...]
        """


        pass


    def point2PointMovement(self, xStart, yStart, zStart, xEnd, yEnd, zEnd):

        pass


