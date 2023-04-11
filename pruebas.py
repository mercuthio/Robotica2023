#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
from MapLib import Map2D


def main(args):
    # Inicializo el robot
    # robot = Robot()

    map_file = "maps/" + args.map
    myMap = Map2D(map_file)

    myMap.drawMap(saveSnapshot=False)

    # myMap.go(robot, 2, 0)

    myMap.fillCostMatrix(0, 0, 7, 0)
    myMap.planPath(0, 0, 7, 0)
    print(myMap.costMatrix)
    print(myMap.currentPath)
    myMap.drawMap(saveSnapshot=False)


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", help="name of the map",
                        type=str, default="mapa1.txt")
    args = parser.parse_args()

    main(args)
