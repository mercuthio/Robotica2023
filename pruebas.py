#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
from MapLib import Map2D
from plotOdometry import generatePlot


def main(args):
    # Inicializo el robot
    # robot = Robot()

    map_file = "maps/" + args.map
    myMap = Map2D(map_file)

    myMap.drawMap(saveSnapshot=False)

    # myMap.go(robot, 2, 0)

    start_pos = [0, 0]
    # finish_pos = [myMap.sizeX - 1, 0]
    finish_pos = [2, 0]

    myMap.fillCostMatrix(start_pos[0], start_pos[1],
                         finish_pos[0], finish_pos[1])
    myMap.planPath(start_pos[0], start_pos[1],
                   finish_pos[0], finish_pos[1])
    print(myMap.costMatrix)
    print(myMap.currentPath)
    myMap.drawMap(saveSnapshot=False)

    if args.plot:
        generatePlot("cadena.txt")


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", help="name of the map",
                        type=str, default="mapaA_CARRERA.txt")
    parser.add_argument(
        "-p", "--plot", help="plot odometry", action='store_true')
    args = parser.parse_args()

    main(args)
