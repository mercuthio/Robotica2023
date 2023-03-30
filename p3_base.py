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

        # 2. Loop running the tracking until reaching the ball, then catch the ball
        targetSize = 230
        target = 320
        res = robot.trackObject(targetSize, target, colorRangeMin=[
                                0, 0, 0], colorRangeMax=[255, 255, 255])

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
