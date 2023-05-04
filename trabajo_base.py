#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import cv2
from cv2 import VideoCapture
import numpy as np
from Robot import Robot
from MapLib import Map2D
from image_match import match_images
from RacePhases import *
import time


TAM_BALDOSAS = 40  # cm
Y_INICAL_CORRECTA = -2 * TAM_BALDOSAS  # cm
DISTANCIA_OPTIMA_PARED = 15


def main(args):
    try:
        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()

        # Launch updateOdometry thread()
        robot.startOdometry()

        # Init gyro and light sensor
        robot.waitGyro()
        robot.waitLight()

        cam = VideoCapture(0)
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")
        print("                    PHASE 0: READ COLOR                ")
        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")

        # Read color from sensor
        color = robot.read_luminosity()
        if color == "White":
            robot.salida = "A"
        else:
            robot.salida = "B"

        print("[c] Color de la cartulina:", color)

        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")
        print("                    PHASE 1: SLALOM                    ")
        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")

        if robot.salida == "A":  # Mapa A
            # Empieza girando a la derecha
            slalom(robot, "A")

        else:  # Mapa B
            # Empieza girando a la derecha
            slalom(robot, "B")

        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")
        print("                  PHASE 2: MAP NAVIGATION              ")
        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")

        if robot.salida == "A":
            mapa = "mapaA_CARRERA.txt"
            start_pos = [1, 2]
            finish_pos = [3, 4]
        else:  # Case Map B
            mapa = "mapaB_CARRERA.txt"
            start_pos = [5, 2]
            finish_pos = [3, 4]

        map_file = "maps/" + mapa
        myMap = Map2D(map_file)
        myMap.go(robot, start_pos[0], start_pos[1],
                 finish_pos[0], finish_pos[1])

        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")
        print("                 PHASE 3: DETECT EXIT                  ")
        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")

        # fix_positiontest(robot)
        fix_position2(robot)

        img_r2 = cv2.imread("imagenes/R2-D2_s.png")
        img_bb8 = cv2.imread("imagenes/BB8_s.png")

        salida = check_output(cam, img_r2, img_bb8, robot.salida)
        print("La salida es por la", salida)
        while salida == "No encontrado":
            # Nos acercamos un poco
            robot.setSpeed(10, 0)
            time.sleep(0.5)
            robot.setSpeed(0, 0)
            salida = check_output(cam, img_r2, img_bb8, robot.salida)
            print("La salida es por la", salida)

        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")
        print("                    PHASE 4: GET BALL                  ")
        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")

        targetSize = 230
        target = 320
        robot.trackObject(cam, targetSize, target, colorRangeMin=[
            0, 0, 0], colorRangeMax=[255, 255, 255])

        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")
        print("                     PHASE 5: LEAVE                    ")
        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")

        # Guardamos la odometría actual del robot (el eje y es el que nos interesa). Con
        # esto, sabemos el error
        _, y_actual, _ = robot.readOdometry()
        error_y = y_actual - Y_INICAL_CORRECTA

        # El robot pilla la pelota... #

        # Nos ponemos mirando a la orientación que toca (dependiendo de donde se
        # encuentre el R2D2 y el BB8)

        if salida == "izquierda":
            robot.turnOdometry(90, -180)
        else:
            robot.turnOdometry(90, 0)

        print("Me he enfilado a la pared")

        # Nos acercamos a la pared hasta la distancia que toca.
        while robot.read_ultrasonic() > 20:
            robot.setSpeed(15, 0)
        robot.setSpeed(0, 0)

        # Corregimos
        # distancia = robot.read_ultrasonic()
        # distancia_optima = 20
        # if abs(robot.read_ultrasonic() - DISTANCIA_OPTIMA_PARED) > 0.5:
        #     time.sleep(1)
        #     robot.setSpeed(distancia - distancia_optima, 0)
        #     time.sleep(1)
        #     robot.setSpeed(0, 0)

        print("Ya estoy cerca")

        # Volvemos a girar para ver hacia el norte
        if salida == "izquierda":
            robot.turnOdometry(-90, 90)
        else:
            robot.turnOdometry(90, -90)
        print("HOLA")

        # Sacamos de la odometría la y actual y la usamos para saber cuánto tenemos que
        # avanzar (la distancia es la posición actual + el error)
        _, y_actual, _ = robot.readOdometry()
        distancia_a_recorrer = abs(y_actual + error_y)
        tiempo_recorrido = distancia_a_recorrer / 20  # segundos

        print("y_actual: ", y_actual)
        print("distancia_a_recorrer: ", distancia_a_recorrer)
        print("tiempo_recorrido: ", tiempo_recorrido)

        # Avanzamos lo necesario.
        robot.setSpeed(20, 0)
        time.sleep(tiempo_recorrido)
        robot.setSpeed(0, 0)

        # # This currently unconfigure the sensors, disable the motors,
        # # and restore the LED to the control of the BrickPi3 firmware.
        robot.setSpeed(0, 0)
        robot.BP.reset_all()
        robot.stopOdometry()

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
