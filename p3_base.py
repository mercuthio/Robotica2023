#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import cv2
import numpy as np
import time
from Robot import Robot

# def centrar_pelota(robot):
#     Ancho_imagen = 640
#     # Mientras no este centrada, girar hasta que este centrada
#     while True:
#         pelota = get_blob()


def main(args):
    try:
        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()

        # 1. launch updateOdometry thread()
        # robot.startOdometry()

        # 2. Loop running the tracking until ??, then catch the ball
        # TO-DO: ADD to the Robot class a method to track an object, given certain parameters
        # for example the different target properties we want (size, position, color, ..)
        # or a boolean to indicate if we want the robot to catch the object or not
        # At least COLOR, the rest are up to you, but always put a default value.
        # res = robot.trackObject(colorRangeMin=[0,0,0], colorRangeMax=[255,255,255],
        #                   targetSize=??, target??=??, ...)
        targetSize = 149
        target = 320
        res = robot.trackObject(targetSize, target, colorRangeMin=[
                                0, 0, 0], colorRangeMax=[255, 255, 255])
        # robot.catch()

        # if res:
        #   robot.catch

        # Compruebo si hay rojo
        # Si no lo hay
        #   Buscar la pelota
        # Si lo hay
        #   Compruebo si hay circulo rojo
        #   Si no lo hay
        #       OK
        #   Si lo hay
        #       Buscar la pelota

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        # robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.setSpeed(0, 0)
        robot.stopOdometry()


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--color", help="color of the ball to track",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)
