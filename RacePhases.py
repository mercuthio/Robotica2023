import numpy as np
import time
import cv2
from Robot import Robot
from cv2 import VideoCapture
from image_match import match_images


def slalom(robot, id):
    """Performs a slalom depending on the id"""
    if id == "A":
        robot.turnOdometry(-90, -180)

        robot.setSpeed(40*np.pi/8, np.radians(180 / 8))
        time.sleep(8)

        robot.setSpeed(40*np.pi/8, np.radians(-180 / 8))
        time.sleep(8)

        robot.setSpeed(0, 0)

        # Ajuste de theta
        robot.turnOdometry(20, -180)

        # Ajustamos con pared 1
        fix_position(robot)
        robot.turnOdometry(90, -90)

        # Ajustamos con pared 2
        fix_position2(robot)
        robot.turnOdometry(90, 0)

        # Con 45 grados
        # robot.turnOdometry(-90, -135)
        # robot.setSpeed(np.pi * np.sqrt((35*35 + 21*21) / 2)/8, np.radians(90 / 8))
        # time.sleep(8)
        # robot.setSpeed(np.pi * np.sqrt((35*35 + 25*25) / 2)/8, np.radians(-90 / 8))
        # time.sleep(8)
        # robot.turnOdometry(20, -180)
        # fix_position(robot)
        # robot.turnOdometry(90, 0)
    else:

        robot.turnOdometry(90, 0)

        robot.setSpeed(40*np.pi/4, np.radians(-180 / 4))
        time.sleep(4)

        robot.setSpeed(0, 0)

        fix_position2(robot)

        robot.setSpeed(40*np.pi/4, np.radians(180 / 4))
        time.sleep(4)

        robot.setSpeed(0, 0)

        # Ajuste de theta
        if robot.read_gyro() < -180:
            robot.turnOdometry(20, -180)
        elif robot.read_gyro() > 180:
            robot.turnOdometry(-20, -180)

        # Ajustamos con pared 1
        fix_position(robot)
        robot.turnOdometry(-90, -90)

        # Ajustamos con pared 1
        fix_position2(robot)
        robot.turnOdometry(-90, -180)


def fix_position(robot):
    distancia = robot.read_ultrasonic()
    distancia_optima = 55

    robot.setSpeed(distancia - distancia_optima, 0)
    time.sleep(1)
    robot.setSpeed(0, 0)


def fix_position2(robot):
    distancia_optima = 15
    distancia = robot.read_ultrasonic() % 40

    robot.setSpeed(distancia - distancia_optima, 0)
    time.sleep(1)
    robot.setSpeed(0, 0)


def get_img(cam):
    _, img = cam.read()
    while img is None:
        _, img = cam.read()
    return img


def check_output(cam, robot_img_r2, robot_img_bb8, mapa):

    test_img = get_img(cam)

    found_r2, w_pos_r2 = match_images(robot_img_r2, test_img)
    found_bb8, w_pos_bb8 = match_images(robot_img_bb8, test_img)

    if found_r2 and found_bb8:
        if mapa == "A":
            return "izquierda" if w_pos_r2 < w_pos_bb8 else "derecha"
        else:
            return "izquierda" if w_pos_bb8 < w_pos_r2 else "derecha"
    else:
        return "No encontrado"
