#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import cv2
from cv2 import VideoCapture
import numpy as np
from Robot import Robot
from MapLib import Map2D
from plotManager import generatePlot
from image_match import match_images


def main(args):
    try:
        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()

        # Launch updateOdometry thread()
        robot.startOdometry()

        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =
        #                     PHASE 0: READ COLOR
        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =

        # Read color from sensor

        robot.salida = "A"

        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =
        #                     PHASE 1: SLALOM
        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =

        # No entiendo la orientacion xd

        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =
        #                     PHASE 2: MAP NAVIGATION
        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =

        if robot.salida == "A":
            mapa = "mapaA_CARRERA.txt"
            start_pos = [1, 2]
            finish_pos = [4, 3]
        else:  # Case Map B
            mapa = "mapaB_CARRERA.txt"
            start_pos = [5, 2]
            finish_pos = [2, 3]

        map_file = "maps/" + mapa
        myMap = Map2D(map_file)
        myMap.go(robot, start_pos[0], start_pos[1],
                 finish_pos[0], finish_pos[1])

        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =
        #                     PHASE 3: DETECT EXIT
        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =

        if robot.salida == "A":
            img_robot = cv2.imread("imagenes/R2-D2_s.png")
        else:
            img_robot = cv2.imread("imagenes/BB8_s.png")

        cam = VideoCapture(0)
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Get 3 images
        _, img = cam.read()
        _, img = cam.read() if img is None else img
        _, img = cam.read() if img is None else img

        match_images(img_robot, img)

        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =
        #                     PHASE 4: GET RED BALL
        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =

        targetSize = 230
        target = 320
        robot.trackObject(targetSize, target, colorRangeMin=[
            0, 0, 0], colorRangeMax=[255, 255, 255])

        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =
        #                     PHASE 4: LEAVE
        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =

        # Hola buenas tardes como estamos

        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =
        #               PHASE 5: KILL PUERRO AND STOP
        # = = = = = = = = = = = = = = = = = = = = = = = = = = = =

        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.setSpeed(0, 0)
        robot.BP.reset_all()
        robot.stopOdometry()

        if args.plot:
            generatePlot(robot.log_file_name)

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.setSpeed(0, 0)
        robot.BP.reset_all()
        robot.stopOdometry()


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p", "--plot", help="plot odometry", action='store_true')
    args = parser.parse_args()

    main(args)
