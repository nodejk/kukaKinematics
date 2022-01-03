# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import numpy as np

from kuka import kuka
from testconfigurations import possibleConfig, inputEndEff

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # print_hi('PyCharm')
    pass
    # kukaBot = kuka()

    # print(kukaBot.forwardKinematics(180, -90, 90, -180, 90, 180))
    # print(kukaBot.forwardKinematics(20, -80, 100, -45, 50, -10))

    # print(kukaBot.forwardKinematics(1.29123907, -100.90206758, 63.85065805, -122.53481659, 101.96362111, -37.67753418))

    # for testing purposes.
    # endEffector = [[ -8.85562315e-01,   2.69213051e-02,  -4.63739830e-01,   9.16922244e+02],
    #                [ -4.47193236e-01,  -3.19530365e-01,   8.35415199e-01,   1.56699319e+02],
    #                [ -1.25688490e-01,   9.47193533e-01,   2.95003075e-01,   2.66453955e+03],
    #                [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,  1.00000000e+00]]

    # # print(kukaBot.)
    # endEffector = np.array(endEffector)
    #
    # # just a test.
    # temp = np.array(kukaBot.inverseKinematicsValidConfigurations(endEffector))
    #
    # count = 0
    # for i in temp:
    #     print(i)
    #
    # print(len(temp))
    # inputEndEff = np.array(inputEndEff)
    # for i in temp:
    #     theta1 = i[0]
    #     theta2 = i[1]
    #     theta3 = i[2]
    #     theta4 = i[3]
    #     theta5 = i[4]
    #     theta6 = i[5]
    #
    #     endEff = kukaBot.forwardKinematics(theta1, theta2, theta3, theta4, theta5, theta6)
    #
    #     print(endEff)
    #     print(i)
    #     print("_-------")
    #     if (np.sum((endEff - inputEndEff) ** 2)<0.001):
    #         count += 1
    #
    # print(count)
    #

    # startPoint = [[-1, 0, 0, -1550],
    #               [0, -1, 0, 0],
    #               [0, 0, 1, 2125],
    #               [0, 0, 0, 1]]
    #
    # startPoint = np.array(startPoint)
    #
    # endPoint = [[-1, 0, 0, -1550],
    #             [0, -1, 0, 100],
    #             [0, 0, 1, 2250],
    #             [0, 0, 0, 1]]
    #
    # endPoint = np.array(endPoint)
    #
    # startingTheta = [180, -90, 90, -180, 90, 180]
    #
    # samplingTime = 12.0 / 1000
    # maxVelocity = 500
    # acceleration = 2000
    #
    # angleDiscreteSteps = kukaBot.line2lineMovement(startingTransformationMatrix=startPoint, endTransformationMatrix=endPoint
    #                                             , maxVelocity=maxVelocity, acceleration=acceleration, startingTheta=startingTheta,
    #                                             samplingTime=samplingTime)
    #
    #
    # for i in angleDiscreteSteps:
    #     print(i)

