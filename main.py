# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

from kuka import kuka

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.
    pass

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # print_hi('PyCharm')

    kukaBot = kuka()

    # print(kukaBot.forwardKinematics(180, -90, 90, -180, 90, 180))
    print(kukaBot.forwardKinematics(20, -80, 100, -45, 50, -10))
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
