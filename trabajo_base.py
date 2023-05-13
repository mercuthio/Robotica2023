#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import cv2
from cv2 import VideoCapture
import numpy as np
from Robot import Robot
from MapLib import Map2D
from image_match import match_images
from get_blob import get_red
from RacePhases import *
import time


TAM_BALDOSAS = 40  # cm
Y_INICAL_CORRECTA = -2 * TAM_BALDOSAS  # cm
DISTANCIA_OPTIMA_PARED = 18


def main(args):
    try:
        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()

        # Do not create log file if told
        if args.log == "False":
            robot.log_file_enabled = False

        # Init gyro, light sensor and sonar
        robot.waitGyro()
        robot.waitLight()

        cam = VideoCapture(0)
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Launch updateOdometry thread()
        robot.startOdometry()

        print("====== BATERIA:", round(100 *
              robot.BP.get_voltage_battery() / 12, 2), "% ======")

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

        robot.BP.set_sensor_type(
            robot.BP.PORT_3, robot.BP.SENSOR_TYPE.NXT_LIGHT_OFF)

        # Esperar input del usuario para comenzar el circuito
        input("[+] Color obtenido, pulse una tecla para continuar...")

        # Avanzamos a la salida inicial
        robot.setSpeed(20, 0)
        time.sleep(1)
        robot.setSpeed(0, 0)

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
            finish_pos = [3, 3]
        else:  # Case Map B
            mapa = "mapaB_CARRERA.txt"
            start_pos = [5, 2]
            finish_pos = [3, 3]

        map_file = "maps/" + mapa
        myMap = Map2D(map_file)
        myMap.go(robot, start_pos[0], start_pos[1],
                 finish_pos[0], finish_pos[1])

        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")
        print("                 PHASE 3: DETECT EXIT                  ")
        print("= = = = = = = = = = = = = = = = = = = = = = = = = = = =")

        if robot.salida == "A":
            robot.turnOdometry(-90, 65)
        else:
            robot.turnOdometry(90, 115)

        robot.setSpeed(20, 0)
        time.sleep(3)
        robot.setSpeed(0, 0)


        img_r2 = cv2.imread("imagenes/R2-D2_s.png")
        img_bb8 = cv2.imread("imagenes/BB8_s.png")

        salida = check_output(cam, img_r2, img_bb8, robot.salida)
        print("La salida es por la", salida)

        contador = 0

        while salida == "No encontrado":
            # Nos acercamos un poco
            robot.setSpeed(10, 0)
            time.sleep(0.5)
            robot.setSpeed(0, 0)
            salida = check_output(cam, img_r2, img_bb8, robot.salida)
            print("=== La salida es por la", salida, "===")
            contador += 1
            if contador >= 5:
                if robot.salida == "A":
                    salida = "izquierda"
                else:
                    salida = "derecha"
                break

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

        blob_red = False

        while blob_red == False:

            # Nos ponemos mirando a la orientación que toca (dependiendo de donde se
            # encuentre el R2D2 y el BB8)

            if salida == "izquierda":
                robot.turnOdometry(90, -180)
                destino = 180
            else:
                robot.turnOdometry(-90, 0)
                destino = 0

            # Nos acercamos a la pared hasta la distancia que toca.
            while robot.read_ultrasonic() > DISTANCIA_OPTIMA_PARED:
                time.sleep(0.01)
                theta = robot.read_gyro()
                theta = (theta + 180) % 360 - 180
                if destino == 180 and theta < 0:
                    destino = -180
                # if destino == 0 and theta < 0:
                #     destino = 180
                #     theta = -theta
                # print("[Recalculando w]:", np.radians((destino_fin - theta) / 1.0), destino_fin)

                # Corregimos la orientación del robot
                if destino == 180 or destino == -180:
                    robot.setSpeed(20, np.radians((destino - theta) / 1.0))
                else:
                    robot.setSpeed(20, 0)
                _, y_actual, _ = robot.readOdometry()
            robot.setSpeed(0, 0)

            blob_red = get_red(cam, False)
            blob_red = get_red(cam, False)
            blob_red = get_red(cam, False)

            # Si ve bastante rojo, la ha cogido
            if blob_red:
                # Pelota conseguida
                print(" === CONFIRMADO. Llevo la pelota. ===")
            # No ha atrapado la pelota
            else:
                print("No he conseguido atrapar la pelota.")
                robot.uncatch()

                robot.trackObject(cam, targetSize, target, colorRangeMin=[
                0, 0, 0], colorRangeMax=[255, 255, 255])

        # Volvemos a girar para ver hacia el oeste
        if salida == "izquierda":
            robot.turnOdometry(-90, 90)
        else:
            robot.turnOdometry(90, 90)

        # Avanzamos hasta alcanzar la salida siguiendo la odometria 
        _, y_actual, _ = robot.readOdometry()
        destino = 90

        # Mientras no lleguemos a la salida
        while y_actual <= 15:
            # robot.setSpeed(20, 0)
            time.sleep(0.01)
            theta = robot.read_gyro()
            theta = (theta + 180) % 360 - 180
            # print("[Recalculando w]:", np.radians((destino - theta) / 1.0), destino)
            
            # Corregimos la orientación del robot
            robot.setSpeed(20, np.radians((destino - theta) / 1.0))
            _, y_actual, _ = robot.readOdometry()

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
    parser.add_argument(
        "-l", "--log", help="save odometry log file", type=str, default="True")
    args = parser.parse_args()

    main(args)
