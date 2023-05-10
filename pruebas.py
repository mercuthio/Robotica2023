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

    # Posicion de la salida respecto al mundo
    WxS = [600, 2800, np.radians(0)]
    WtS = hom_array(WxS)

    # Posicion del robot respecto a la salida
    # SxR = [-40.96 * 10, -33.35 * 10, np.radians(-98.71)]
    # StR = hom_array(SxR)

    # WtR = np.dot(WtS, StR)
    # WxR = loc_array(WtR)

    # robotPosVectors = []

    # robotPosVectors.append(WxR)

    with open(odometry, "r") as archivo:
        lineas = archivo.readlines()

    robotPosVectors = []
    for line in lineas:
        x_pos = line.find("X:")
        y_pos = line.find("Y:")
        th_pos = line.find("TH:")
        v_pos = line.find("V:")

        x_val = float(line[x_pos+2:y_pos].strip())
        y_val = float(line[y_pos+2:th_pos].strip())
        th_val = float(line[th_pos+3:v_pos].strip())

        # Posicion del robot respecto a la salida
        SxR = [x_val * 10, y_val * 10, np.radians(th_val)]
        StR = hom_array(SxR)

        WtR = np.dot(WtS, StR)
        WxR = loc_array(WtR)

        robotPosVectors.append(WxR)

    myMap.drawMapWithRobotLocations(robotPosVectors, saveSnapshot=False)

    # myMap.drawMap(saveSnapshot=False)

    # if args.plot:
    #     generatePlot("cadena.txt")


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
