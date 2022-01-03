import sys

import numpy as np
import math


def dh_matrix(theta, alpha, r, d):
    """
    This function is used to form the transformation matrix required for the DH table.
    :param theta: angle of rotation around z(i).
    :param alpha: angle of rotation around x(i+1) to reach x(i)
    :param r: displacement from i to i+1 frame along the x(i+1) axis.
    :param d: displacement from i to i+1 frame along the z(i) axis.
    :return: 4X4 transformation matrix containing the orientation and the position of the i+1 w.rt. i frame
                according to the given parameters.
    """
    theta_ = math.radians(theta)
    alpha_ = math.radians(alpha)

    a = np.matrix([[math.cos(theta_), -1 * math.sin(theta_) * math.cos(alpha_), math.sin(theta_) * math.sin(alpha_),
                    r * math.cos(theta_)],
                   [math.sin(theta_), math.cos(theta_) * math.cos(alpha_), -1 * math.cos(theta_) * math.sin(alpha_),
                    r * math.sin(theta_)],
                   [0, math.sin(alpha_), math.cos(alpha_), d],
                   [0, 0, 0, 1]])

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


def getCosineAngle(a, b, c):
    """
    this function returns the angle between sides a and b of a triangle.
    :param a: length of triangle with side a.
    :param b: another length of triangle with side a.
    :param c: the side opposite to the angle.
    :return: angle in degrees.
    """
    cos1 = (a * a + b * b - c * c) / (2 * a * b)

    return math.degrees(math.acos(cos1))


class kuka:
    """
    This implementation is for the 6-DOF robot Implementation of Forwards Kinematics, Inverse Kinematics,
    Point to Point Movement and Synchronous movement for Kuka KR 120 R2700-2.

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

    def thetaValidConfiguration(self, joint, theta, constraint_type = "angle"):
        """
        This method is used to check if a theta configuration is valid or not. These constraints are given in the
        datasheet and are used in lineMovement method.
        :param joint: joint for which the constraints have to be checked.
        :param theta: angle of joint in degrees.
        :param constraint_type: "angle" for range of motion possible for the joint1 and "speed" if the given speed is
        possible for the joint to obtain.
        :return: boolean. true if the configuration is valid otherwise false.
        """

        if constraint_type not in ['angle', 'speed']:
            raise Exception("please enter valid constraint type, (angle or speed) for joint {}".format(joint))

        if joint not in [1, 2, 3, 4, 5, 6]:
            raise Exception("please enter a valid link number. Given: {}".format(joint))

        if joint == 1:
            if constraint_type == "angle":
                if abs(theta) <= 185:
                    return True
                else:
                    return False
            elif constraint_type == "speed":
                if abs(theta) <= 120:
                    return True
                else:
                    return False
        elif joint == 2:
            if constraint_type == "angle":
                if theta >= -140 and theta <= -5:
                    return True
                else:
                    return False
            elif constraint_type == "speed":
                if abs(theta) <= 115:
                    return True
                else:
                    return False
        elif joint == 3:
            if constraint_type == "angle":
                if theta >= -120 and theta <= 168:
                    return True
                else:
                    return False
            elif constraint_type == "speed":
                if abs(theta) <= 120:
                    return True
                else:
                    return False
        elif joint == 4:
            if constraint_type == "angle":
                if abs(theta) <= 350:
                    return True
                else:
                    return False
            elif constraint_type == "speed":
                if abs(theta) <= 190:
                    return True
                else:
                    return False
        elif joint == 5:
            if constraint_type == "angle":
                if abs(theta) <= 125:
                    return True
                else:
                    return False
            elif constraint_type == "speed":
                if abs(theta) <= 180:
                    return True
                else:
                    return False
        elif joint == 6:
            if constraint_type == "angle":
                if abs(theta) <= 350:
                    return True
                else:
                    return False
            elif constraint_type == "speed":
                if abs(theta) <= 260:
                    return True
                else:
                    return False

    def __theta1Configurations(self, xWrist, yWrist, zWrist):
        """
        This method returns 3 configurations for the first joint.
        :param flange: final 4 X 4 matrix of the end effector
        :return: [theta1.1, theta1.2, theta1.3] angles in degrees.
        """

        theta1 = math.degrees(math.atan2(yWrist, xWrist))

        if yWrist >= 0 and xWrist < 0:
            return [180 - theta1, (180 - theta1) + 180, (180 - theta1) - 180]
        elif yWrist >= 0 and xWrist > 0:
            return [-theta1, -theta1 - 180, 180 - theta1]
        elif yWrist <= 0 and xWrist < 0:
            return [-(180 + theta1), -theta1, -180 - (180 + theta1)]
        elif yWrist <= 0 and xWrist > 0:
            return [-theta1, -(theta1 - 180), -(180 + theta1)]
        elif xWrist == 0:
            return [90, -90]

    def __getLink2Position(self, theta1):
        """
        This method is used to find the position and orientation for the 2nd link using forward kinematics.
        This is important in order to calculate 3rd and 2nd configurations.
        :param theta1: theta1 angle
        :return: 4X4 transformation matrix in form of numpy array.
        """

        a1 = dh_matrix(0, self.alpha0, self.a0, self.d0)
        a2 = dh_matrix(theta1, self.alpha1, self.a1, self.d1)

        return np.matmul(a1, a2)

    def __getLink5Position(self, theta1, theta2, theta3):
        """
        This method is used to find the end position and orientation of the link5 using forward kinematics
        :param theta1: angle of link 1.
        :param theta2: angle of link 2.
        :param theta3: angle of link 3.
        :return: 4X4 transformation matrix in form of numpy array.
        """

        a1 = dh_matrix(0, -180, 0, 645)
        a2 = dh_matrix(theta1, 90, 330, 0)
        a3 = dh_matrix(theta2, 0, 1150, 0)
        a4 = dh_matrix(theta3 - 90, 90, 115, 0)

        return np.matmul(np.matmul(np.matmul(a1, a2), a3), a4)

    def __getwristConfigurations(self, theta4, theta5, theta6):
        """
        This method is used to find all the configurations possible.
        :param theta4: angle of link 4.
        :param theta5: angle of link 5.
        :param theta6: angle of link 6.
        :return: [[theta4.1, theta5.1, theta6.1], [theta4.1, theta5.2, theta6.2], ...]
        """
        outputConfig = []

        outputConfig.append([theta4, theta5, theta6])

        if theta4 > 0:
            if theta6 > 0:
                outputConfig.append([theta4 - 360, theta5, theta6])
                outputConfig.append([theta4 - 360, theta5, theta6 - 360])
                outputConfig.append([theta4, theta5, theta6 - 360])
            else:
                outputConfig.append([theta4 - 360, theta5, theta6])
                outputConfig.append([theta4 - 360, theta5, theta6 + 360])
                outputConfig.append([theta4, theta5, theta6 + 360])
        else:
            if theta6 > 0:
                outputConfig.append([theta4 + 360, theta5, theta6])
                outputConfig.append([theta4 + 360, theta5, theta6 - 360])
                outputConfig.append([theta4, theta5, theta6 - 360])
            else:
                outputConfig.append([theta4 + 360, theta5, theta6])
                outputConfig.append([theta4 + 360, theta5, theta6 + 360])
                outputConfig.append([theta4, theta5, theta6 + 360])

        return outputConfig

    def __endOrientation(self, configurations, endEffector):
        """
        This method returns all 48 configurations (at max).
        :param configurations: configurations calculated previously
        [[theta1.1, theta2.1, theta3.1], [theta1.1, theta2.2, theta3.2], ...]
        :param endEffector: transformation matrix of the end effector (not the wrist position)
        :return: list of 48 configurations at max.
        [[theta1.1, theta2.1, theta3.1, theta4.1, theta5.1, theta6.1],
        [theta1.1, theta2.1, theta3.1, theta4.1, theta5.2, theta6.2],...]
        """
        outputConfigurations = []
        endEffectorRotationMatrix = endEffector[0:3, 0:3]

        for configuration in configurations:

            theta1, theta2, theta3 = configuration[0], configuration[1], configuration[2]

            transformationMatrix = self.__getLink5Position(theta1, theta2, theta3)

            rotationMatrix = np.linalg.inv(transformationMatrix[0:3, 0:3])

            armEndEff = np.matmul(rotationMatrix, endEffectorRotationMatrix)

            R02 = armEndEff[0, 2]
            R12 = armEndEff[1, 2]
            R20 = armEndEff[2, 0]
            R21 = armEndEff[2, 1]
            R22 = armEndEff[2, 2]

            theta5 = math.degrees(math.atan2(pow(R02 * R02 + R12 * R12, 0.5), -R22))

            # final wrist orientation

            finalwristOrientation = []
            # in order to handle singularity, as the solutions can be infinite.
            if theta5 < 0.0001 or abs(abs(theta5) - 180) < 0.0001:
                finalwristOrientation = [[0, 0, 0], [0, 0, 180], [180, 0, 0], [180, 0, 180]]
            else:
                # since there are 2 configurations for the link 5.

                theta5 = math.degrees(math.atan2(pow(R02 * R02 + R12 * R12, 0.5), -R22))
                theta4 = math.degrees(math.atan2(-R12, -R02))
                theta6 = math.degrees(math.atan2(R21, R20))
                # print("---")
                # print(self.__getwristConfigurations(theta4, theta5, theta6))
                finalwristOrientation += self.__getwristConfigurations(theta4, theta5, theta6)

                theta5 = math.degrees(math.atan2(-pow(R02 * R02 + R12 * R12, 0.5), -R22))
                theta4 = math.degrees(math.atan2(R12, R02))
                theta6 = math.degrees(math.atan2(-R21, -R20))

                finalwristOrientation += self.__getwristConfigurations(theta4, theta5, theta6)

            for wristConfiguration in finalwristOrientation:
                tempConfig = configuration + wristConfiguration
                outputConfigurations.append(tempConfig)

        return outputConfigurations

    def __theta2Configurations(self, theta1Configurations, xWrist, yWrist, zWrist):
        """
        This method returns the 6 (at max) configurations for elbow up and elbow down.
        :param theta1Configurations: list of theta1 configurations.
        :param xWrist: x coordinate of the wrist.
        :param yWrist: y coordinate of the wrist.
        :param zWrist: z coordinate of the wrist.
        :return: [[theta1.1, theta2.1, mark], [theta1.1, theta2.2, mark], ...]
        mark = 1 for elbow up configuration.
        mark = 0 for elbow down configuration.
        This mark is used to distinguish the 2 configurations.

        Refer to inverse kinematics for 2 links for additional info.
        """
        outputConfigurations = []

        wristCoordinates = np.expand_dims(np.array([xWrist, yWrist, zWrist, 1]), axis=1)

        ld = getHypotenuse(self.d4, self.a3)
        l2 = abs(self.a2)

        for _, theta1 in enumerate(theta1Configurations):

            link2TransformationMatrix = self.__getLink2Position(theta1)

            link2TranforsmationMatrix = np.linalg.inv(link2TransformationMatrix)

            # get coordinates of the wrist point w.r.t to link2.
            wristCoordinatesLink2 = np.matmul(link2TranforsmationMatrix, wristCoordinates)

            # distance of wrist point from link2.
            hypotenuseLength = getHypotenuse(wristCoordinatesLink2[0].item(), wristCoordinatesLink2[1].item())

            beta1 = math.degrees(math.atan2(wristCoordinatesLink2[1], wristCoordinatesLink2[0]))

            # since not every configuration is possible for theta2.
            try:
                gamma1 = getCosineAngle(l2, hypotenuseLength, ld)

                elbowUp = beta1 + gamma1
                elbowDown = beta1 - gamma1

                # this is just angle adjustment according to the project's desired configuration.
                # found this with experimentation.

                if -elbowDown - 180 > 0:
                    elbowDown += 360
                if elbowUp - 180 > 0:
                    elbowUp -= 360

                outputConfigurations.append([theta1, elbowUp, 1])
                outputConfigurations.append([theta1, elbowDown, 0])

            except:
                pass

        return outputConfigurations

    def __theta3Configurations(self, theta2Configurations, xWrist, yWrist, zWrist):
        """
        This method is used to find the third configurations.
        :param theta2Configurations: list of configurations found in method __theta2Configurations.
        :param xWrist: x coordinate of the wrist.
        :param yWrist: y coordinate of the wrist.
        :param zWrist: z coordinate of the wrist.
        :return: [[theta1.1, theta2.1, theta3.1], [theta1.1, theta2.2, theta3.2], ....]
        """

        outputConfigurations = []

        wristCoordinates = np.expand_dims(np.array([xWrist, yWrist, zWrist, 1]), axis=1)

        # shoulder offset.
        phi = math.degrees(math.atan2(self.a3, abs(self.d4)))

        l2 = abs(self.a2)
        ld = getHypotenuse(self.d4, self.a3)

        for _, thetaConfiguration in enumerate(theta2Configurations):
            theta1 = thetaConfiguration[0]
            theta2 = thetaConfiguration[1]
            elbow = thetaConfiguration[2]

            link2TransformationMatrix = self.__getLink2Position(theta1)
            link2TransformationMatrix = np.linalg.inv(link2TransformationMatrix)

            # get coordinates of the wrist point w.r.t to link2.
            wristCoordinatesLink2 = np.matmul(link2TransformationMatrix, wristCoordinates)
            hypotenuseLength = getHypotenuse(wristCoordinatesLink2[0].item(), wristCoordinatesLink2[1].item())

            try:
                eta = getCosineAngle(l2, ld, hypotenuseLength)

                if elbow == 1:

                    outputConfigurations.append([theta1, theta2, -180 + phi + eta])
                else:
                    theta3 = 180 + phi - eta

                    if theta3 > 180:
                        theta3 = phi - eta - 180

                    outputConfigurations.append([theta1, theta2, theta3])
            except:
                pass

        return outputConfigurations

    def inverseKinematicsAllConfig(self, endTransformationMatrix):
        """
        This method is used for the inverse kinematics.
        This calculates all the configurations for a given end effector position and orientation
        using geometry.
        Note that these might be all the valid possible configurations for the robot
        because of physical constraints.
        Additionally, this method returns configurations according to the direction of the robot
        defined in the datasheet.
        :param endTransformationMatrix: 4X4 end transformation matrix as a form of numpy array.
        :return: [[theta1, theta2,...theta6], [theta1, theta2,...theta6]...]
        """
        xWrist, yWrist, zWrist = self.getWristPosition(endTransformationMatrix)

        theta1Configurations = self.__theta1Configurations(xWrist, yWrist, zWrist)

        theta2Configurations = self.__theta2Configurations(theta1Configurations, xWrist, yWrist, zWrist)

        theta3Configurations = self.__theta3Configurations(theta2Configurations, xWrist, yWrist, zWrist)

        allConfigurations = self.__endOrientation(theta3Configurations, endTransformationMatrix)

        return allConfigurations


    def inverseKinematicsValidConfigurations(self, endTransformationMatrix):
        """
        This method returns the list of valid configurations derived from inverse kinematics.
        :param endTransformationMatrix: 4X4 end transformation matrix as a form of numpy array.
        :return: [[theta1, theta2,...theta6], [theta1, theta2,...theta6]...]
        """
        possibleConfigurations = self.inverseKinematicsAllConfig(endTransformationMatrix)

        validConfigurations = self.__checkConfiguration(possibleConfigurations)

        return validConfigurations

    def __getPoints(self, startPoint, endPoint, d):
        """
        This code find the point which is d distance away from the starting point
        (startPoint) along the line from startPoint to endPoint.
        :param startPoint: list [x1, y1, z1]
        :param endPoint: list [x2, y2, y3]
        :param d: distance from the staring point.
        :return: np array of (4,) [x, y, z, 1].
        """
        x0, y0, z0 = startPoint[0], startPoint[1], startPoint[2]
        x1, y1, z1 = endPoint[0], endPoint[1], endPoint[2]

        distPoints = magnitudeVector(x1 - x0, y1 - y0, z1 - z0)

        x = x0 + d * (x1 - x0) / distPoints
        y = y0 + d * (y1 - y0) / distPoints
        z = z0 + d * (z1 - z0) / distPoints

        return np.array([x, y, z, 1])


    def __getDiscreteCartesianSteps(self, startPoint, endPoint, maxVelocity, acceleration, samplingTime):
        """
        This method returns a list of discrete cartesian steps required to reach from start point to end point.
        This method uses either triangular or trapezoid.
        :param startPoint: [xStart, yStart, zStart] cartesian point
        :param endPoint: [xEnd, yEnd, zEnd] cartesian point.
        :param maxVelocity: maximum velocity that the robot in the
        :param acceleration: acceleration of the robot in the cartesian.
        :param samplingTime: sampling time of the robot.
        :return:
        """

        tc = maxVelocity / acceleration

        xStart, yStart, zStart = startPoint[0], startPoint[1], startPoint[2]
        xEnd, yEnd, zEnd = endPoint[0], endPoint[1], endPoint[2]

        # cartesian distance between the 2 given points.
        distanceBetweenPoints = magnitudeVector(xStart - xEnd, yStart - yEnd, zStart - zEnd)

        velocityReached = pow(distanceBetweenPoints * acceleration, 0.5)

        startPoint = np.array(startPoint)
        endPoint = np.array(endPoint)

        coordinates = []

        if maxVelocity < velocityReached:
            """
            this is the first case where the trapezoid velocity profile is formed.
            """
            totalTime = distanceBetweenPoints / velocityReached + maxVelocity / acceleration
            totalTimeSteps = math.ceil(totalTime / samplingTime)

            for i in range(totalTimeSteps):
                timeStep = i * samplingTime

                if timeStep < tc:
                    discreteStep = 0.5 * acceleration * timeStep * timeStep
                elif timeStep < totalTime - tc:
                    discreteStep = acceleration * tc * (timeStep - 0.5 * tc)
                else:
                    discreteStep = distanceBetweenPoints - 0.5 * acceleration * (timeStep - totalTime) * (timeStep - totalTime)

                coordinateStep = self.__getPoints(startPoint, endPoint, discreteStep)

                coordinates.append(coordinateStep)
        else:
            """
            this is the second case where triangular velocity profile is formed.
            """
            totalTime = pow(distanceBetweenPoints / acceleration, 0.5) * 2
            timeTotalSteps = math.ceil(totalTime / samplingTime)

            for i in range(timeTotalSteps):
                timeStep = i * samplingTime

                if timeStep < totalTime / 2:
                    discreteStep = 0.5 * acceleration * timeStep * timeStep
                else:
                    discreteStep = distanceBetweenPoints - 0.5 * (timeStep - totalTime) * (timeStep - totalTime) * acceleration

                coordinateStep = self.__getPoints(startPoint, endPoint, discreteStep)

                coordinates.append(coordinateStep)

                print(coordinateStep)

        return coordinates

    def __checkConfiguration(self, allThetaConfig):
        """
        From a given set of configurations, it is used to find the configuration which is physically possible for the
        robot to achieve.
        :param allThetaConfig: the list of configurations from the inverse kinematics method
        [[theta1, theta2, theta3,...], [theta1, theta2,...],...]
        :return: valid list of configurations which are possible in the above format.
        """
        outputConfig = []

        for configuration in allThetaConfig:
            t1 = self.thetaValidConfiguration(1, configuration[0], constraint_type="angle")
            t2 = self.thetaValidConfiguration(2, configuration[1], constraint_type="angle")
            t3 = self.thetaValidConfiguration(3, configuration[2], constraint_type="angle")
            t4 = self.thetaValidConfiguration(4, configuration[3], constraint_type="angle")
            t5 = self.thetaValidConfiguration(5, configuration[4], constraint_type="angle")
            t6 = self.thetaValidConfiguration(6, configuration[5], constraint_type="angle")

            if t1 and t2 and t3 and t4 and t5 and t6:
                outputConfig.append(configuration)

        return outputConfig

    def __findOptimalTheta(self, lastThetaConfiguration, validThetaConfigurations, timeSample):
        """
        This method is used to find the optimal theta from a given set of configurations. This method finds the optimal
        configuration by checking if the joint speed is plausible considering the previous joint. Note: this method does
        not take singularities into account.
        :param lastThetaConfiguration: last configuration in which the robot was.
        :param validThetaConfigurations: the number of valid configurations for a given position (from inverse
        kinematics)
        :param timeSample: sampling period for the robot.
        :return: valid configuration in the form of list: [theta1, theta2, theta3, theta4, theta5, theta6]
        """
        thetaPrev = np.array(lastThetaConfiguration)
        validThetaConfigurations = np.array(validThetaConfigurations)

        currentConfig = None

        for configuration in validThetaConfigurations:
            t1Speed = self.thetaValidConfiguration(1, (configuration[0] - thetaPrev[0])/timeSample, constraint_type="speed")
            t2Speed = self.thetaValidConfiguration(2, (configuration[1] - thetaPrev[1])/timeSample, constraint_type="speed")
            t3Speed = self.thetaValidConfiguration(3, (configuration[2] - thetaPrev[2])/timeSample, constraint_type="speed")
            t4Speed = self.thetaValidConfiguration(4, (configuration[3] - thetaPrev[3])/timeSample, constraint_type="speed")
            t5Speed = self.thetaValidConfiguration(5, (configuration[4] - thetaPrev[4])/timeSample, constraint_type="speed")
            t6Speed = self.thetaValidConfiguration(6, (configuration[5] - thetaPrev[5])/timeSample, constraint_type="speed")

            if t1Speed and t2Speed and t3Speed and t4Speed and t5Speed and t6Speed:
                currentConfig = configuration
                break

        if currentConfig is None:
            raise Exception("The linear movement is not possible")

        return currentConfig

    def line2lineMovement(self, startingTransformationMatrix,
                                endTransformationMatrix,
                                samplingTime,
                                maxVelocity,
                                acceleration,
                                startingTheta):
        """
        This method is used to find configurations required to reach from the start to end in a linear
        movement. This method does not change the orientation of the end effector throughout the movement but moves it
        linearly in the cartesian space.
        :param startingTransformationMatrix: start transformation matrix.
        :param endTransformationMatrix: end transformation matrix.
        :param samplingTime: sampling time for the robot.
        :param maxVelocity: max velocity of the end effector in the cartesian space.
        :param acceleration: acceleration of the end effector in the cartesian space.
        :param startingTheta: starting configuration: [theta1, theta2, theta3, theta4, theta5, theta6]
        :return:
        """
        xStart, yStart, zStart = startingTransformationMatrix[0, 3], startingTransformationMatrix[1, 3], \
                                 startingTransformationMatrix[2, 3]

        xEnd, yEnd, zEnd = endTransformationMatrix[0, 3], endTransformationMatrix[1, 3], endTransformationMatrix[2, 3]

        configurationSteps = []

        rotationMatrixEndEff = startingTransformationMatrix[:4, :3]

        coordinateStep = self.__getDiscreteCartesianSteps([xStart, yStart, zStart],
                                                       [xEnd, yEnd, zEnd],
                                                       maxVelocity,
                                                       acceleration,samplingTime)

        for num, coordinateStep in enumerate(coordinateStep):
            point1 = np.expand_dims(np.array(coordinateStep), axis= 1)

            transformationMatrix = np.concatenate([rotationMatrixEndEff, point1], axis = 1)

            allThetaConfig = self.inverseKinematicsAllConfig(transformationMatrix)
            validThetaConfig = self.__checkConfiguration(allThetaConfig)

            optimalTheta = self.__findOptimalTheta(startingTheta, validThetaConfig, samplingTime)

            configurationSteps.append(optimalTheta)

            startingTheta = optimalTheta

        return configurationSteps


