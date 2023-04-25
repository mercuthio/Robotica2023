import numpy as np
import time
from Robot import Robot


def slalom(robot, id):
    """Performs a slalom depending on the id"""
    if id == "A":
        robot.turnOdometry(-90, -90)

        robot.setSpeed(40*np.pi/4, np.radians(180 / 4))
        time.sleep(4)
        robot.setSpeed(40*np.pi/4, np.radians(-180 / 4))
        time.sleep(4)

        robot.turnOdometry(90, 0)

    else:
        robot.turnOdometry(90, 90)

        robot.setSpeed(40*np.pi/4, np.radians(-180 / 4))
        time.sleep(4)
        robot.setSpeed(40*np.pi/4, np.radians(180 / 4))
        time.sleep(4)

        robot.turnOdometry(-90, 0)
