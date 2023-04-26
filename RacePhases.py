import numpy as np
import time
from Robot import Robot

def slalom(robot, id):
    """Performs a slalom depending on the id"""
    if id == "A":
        robot.turnOdometry(-90, -90)

        robot.setSpeed(40*np.pi/8, np.radians(180 / 8))
        time.sleep(8)
        robot.setSpeed(40*np.pi/8, np.radians(-180 / 8))
        time.sleep(8)

        robot.turnOdometry(20, -90)
        fix_position(robot)
        robot.turnOdometry(90, 90)
        robot.BP.reset_sensor(robot.BP.PORT_1)

    else:
        robot.turnOdometry(90, 90)

        robot.setSpeed(40*np.pi/8, np.radians(-180 / 8))
        time.sleep(8)
        robot.setSpeed(40*np.pi/8, np.radians(180 / 8))
        time.sleep(8)

        robot.turnOdometry(-10, -90)
        fix_position(robot)
        robot.turnOdometry(90, 90)
        robot.BP.reset_sensor(robot.BP.PORT_1)


def fix_position(robot):
    distancia = robot.read_ultrasonic()
    distancia_optima = 55

    robot.setSpeed(distancia - distancia_optima, 0)
    time.sleep(1)
    robot.setSpeed(0, 0)
