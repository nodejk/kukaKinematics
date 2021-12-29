# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import numpy as np

from kuka import kuka
from testconfigurations import possibleConfig

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # print_hi('PyCharm')

    kukaBot = kuka()

    # print(kukaBot.forwardKinematics(180, -90, 90, -180, 90, 180))
    # print(kukaBot.forwardKinematics(20, -80, 100, -45, 50, -10))

    # print(kukaBot.forwardKinematics(1.29123907, -100.90206758, 63.85065805, -122.53481659, 101.96362111, -37.67753418))

    # for testing purpose.
    endEffector = [[ -8.85562315e-01,   2.69213051e-02,  -4.63739830e-01,   9.16922244e+02],
                   [ -4.47193236e-01,  -3.19530365e-01,   8.35415199e-01,   1.56699319e+02],
                   [ -1.25688490e-01,   9.47193533e-01,   2.95003075e-01,   2.66453955e+03],
                   [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,  1.00000000e+00]]
    # print(kukaBot.)
    endEffector = np.array(endEffector)

    # just a test.
    temp = np.array(kukaBot.inverseKinematicsAllConfig(endEffector))

    count = 0

    possibleConfig = np.array(possibleConfig)
    for i in temp:
        for j in possibleConfig:
            if(np.sum((i-j) ** 2) < 0.00001):
                count += 1
                print("cal: ", i)
                break

    print(count)

