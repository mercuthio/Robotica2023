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

        robot.setSpeed(40*np.pi/4.0, np.radians(180 / 4.0))
        time.sleep(4.0)

        robot.setSpeed(0, 0)

        # Ajuste de theta
        theta = robot.read_gyro()
        theta = (theta + 180) % 360 - 180

        if theta > 0 and theta <= 90:
            robot.turnOdometry(-20, 0)
        elif theta >= -90 and theta < 0:
            robot.turnOdometry(20, 0)

        fix_position2(robot)

        robot.setSpeed(40*np.pi/4.0, np.radians(-180 / 4.0))
        time.sleep(4.0)

        robot.setSpeed(0, 0)

        # Ajuste de theta
        th =  robot.read_gyro()
        if th > -180 and th < -90:
            robot.turnOdometry(-20, -180)
        elif th > -180 and th < 0:
            robot.turnOdometry(20, -180)

        # Ajustamos con pared 1
        # fix_position(robot)
        # robot.turnOdometry(90, -90)

        # Ajustamos con pared 2
        fix_position2(robot)
        robot.turnOdometry(90, 0)
    else:

        robot.turnOdometry(90, 0)

        robot.setSpeed(40*np.pi/4.0, np.radians(-180 / 4.0))
        time.sleep(4.0)

        robot.setSpeed(0, 0)

        theta = robot.read_gyro()
        theta = (theta + 180) % 360 - 180

        if theta >= 90 and theta <= 179:
            robot.turnOdometry(20, -180)
        elif theta > -180 and theta < -90:
            robot.turnOdometry(-20, -180)

        fix_position2(robot)

        robot.setSpeed(40*np.pi/4, np.radians(180 / 4))
        time.sleep(4)

        robot.setSpeed(0, 0)

        # Ajuste de theta
        theta = robot.read_gyro()
        theta = (theta + 180) % 360 - 180

        if theta > 0 and theta <= 90:
            robot.turnOdometry(-20, 0)
        elif theta >= -90 and theta < 0:
            robot.turnOdometry(20, 0)

        # Ajustamos con pared 1
        # fix_position(robot)
        # robot.turnOdometry(-90, -90)

        # Ajustamos con pared 1
        fix_position2(robot)
        robot.turnOdometry(-90, -180)


def fix_position(robot):
    '''Fixes the position of the robot with the noth or south wall after the slalom'''
    distancia = robot.read_ultrasonic()
    distancia_optima = 55

    robot.setSpeed((distancia - distancia_optima) * 2, 0)
    time.sleep(0.5)
    robot.setSpeed(0, 0)


def fix_position2(robot):
    '''Fixes the position of the robot with the east wall after the slalom'''
    distancia_optima = 15
    distancia = robot.read_ultrasonic() % 40

    robot.setSpeed((distancia - distancia_optima) * 2, 0)
    time.sleep(0.5)
    robot.setSpeed(0, 0)


def get_img(cam):
    '''Takes an image with the camera'''
    _, img = cam.read()
    while img is None:
        _, img = cam.read()
    return img

# Devuelve el lado en el que se encuentra el robot a buscar.
# En caso de no encontrar a los dos devuelve "No encontrado"
def check_output(cam, robot_img_r2, robot_img_bb8, mapa):
    '''Returns whether the robot is on the left or the right '''
    test_img = get_img(cam)

    # la funcion match_images calcula el punto medio del cuadrado de homografía
    # y devuelve su componente x
    found_r2, w_pos_r2 = match_images(robot_img_r2, test_img)
    found_bb8, w_pos_bb8 = match_images(robot_img_bb8, test_img)

    if found_r2 and found_bb8:
        if mapa == "A":
            return "izquierda" if w_pos_r2 < w_pos_bb8 else "derecha"
        else:
            return "izquierda" if w_pos_bb8 < w_pos_r2 else "derecha"
    else:
        return "No encontrado"
