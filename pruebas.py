#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
from auxFunc import *
from MapLib import Map2D
# from plotOdometry import generatePlot


def main(args):
    # Inicializo el robot
    # robot = Robot()

    map_file = "maps/" + args.map
    myMap = Map2D(map_file)
    odometry = "logs/" + "log-14h-28m-09s.txt"

    xRM = [600, 2600, np.radians(-90)]
    tRM = hom_array(xRM)

    robotPosVectors = []

    with open(odometry, "r") as archivo:
        lineas = archivo.readlines()

    for linea in lineas:
        # Separar los campos de cada línea
        campos = linea.split("\t")
        x = float(campos[1].split(":")[1])
        y = float(campos[2].split(":")[1])
        th = float(campos[3].split(":")[1])
        punto = np.dot(tRM, [x, y, th])

        robotPosVectors.append(punto)

    myMap.drawMapWithRobotLocations(robotPosVectors, saveSnapshot=False)

    # myMap.drawMap(saveSnapshot=False)

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
