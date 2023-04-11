#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import cv2
import numpy as np
import time
from Robot import Robot
from MapLib import Map2D
from plotOdometry import generatePlot


def main(args):
    try:
        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()

        # 1. launch updateOdometry thread()
        robot.startOdometry()

        map_file = "maps/" + args.map
        myMap = Map2D(map_file)

        # myMap.drawMap(saveSnapshot=False)

        myMap.go(robot, 2, 0)

        # myMap.fillCostMatrix(0, 0, 2, 0)
        # myMap.planPath(0, 0, 2, 0)
        # print(myMap.costMatrix)
        # print(myMap.currentPath)
        # myMap.drawMap(saveSnapshot=False)

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
    parser.add_argument("-m", "--map", help="name of the map",
                        type=str, default="mapa1.txt")
    parser.add_argument(
        "-p", "--plot", help="plot odometry", action='store_true')
    args = parser.parse_args()

    main(args)
