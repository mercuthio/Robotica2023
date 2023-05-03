import numpy as np
import time
from Robot import Robot
from image_match import match_images


def slalom(robot, id):
    """Performs a slalom depending on the id"""
    if id == "A":
        robot.turnOdometry(-90, -180)

        robot.setSpeed(40*np.pi/8, np.radians(180 / 8))
        time.sleep(8)
        robot.setSpeed(40*np.pi/8, np.radians(-180 / 8))
        time.sleep(8)
        robot.turnOdometry(20, -180)
        fix_position(robot)
        robot.turnOdometry(90, 0)
    else:

        robot.setSpeed(40*np.pi/8, np.radians(-180 / 8))
        time.sleep(8)
        robot.setSpeed(40*np.pi/8, np.radians(180 / 8))
        time.sleep(8)

        robot.turnOdometry(-10, 0)
        fix_position(robot)
        robot.turnOdometry(-90, -180)


def fix_position(robot):
    distancia = robot.read_ultrasonic()
    distancia_optima = 55

    robot.setSpeed(distancia - distancia_optima, 0)
    time.sleep(1)
    robot.setSpeed(0, 0)


def check_output(robot_img, test_img):
    found, w_pos = match_images(robot_img, test_img)
    if found == True:
        image_width = test_img.shape[1]
        print(w_pos, image_width / 2)
        return "izquierda" if w_pos < (image_width / 2) else "derecha"
