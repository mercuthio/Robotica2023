#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
import math
from Robot import Robot


def prueba_0(robot):
    robot.setSpeed(40/4, 0)
    time.sleep(4)
    robot.setSpeed(0, 0)
    # robot.setSpeed(0, np.radians(-90))
    # time.sleep(12)
    # robot.setSpeed(0, 0)

def girar_odometria(robot, grados, destino):
    robot.setSpeed(0, np.radians(grados))
    pos = robot.readOdometry()
    
    while abs(pos[2] - destino) > 5:
        pos = robot.readOdometry()
    robot.setSpeed(0, 0)

# Desplazo el robot hasta que su odometría coincida con el destino
def desplazamiento(robot, movimiento, destino):
    robot.setSpeed(movimiento[0], movimiento[1])
    pos = robot.readOdometry()
    while (abs(abs(pos[0]) - abs(destino[0])) > 1.5) or (abs(abs(pos[1]) - abs(destino[1])) > 1.5):
        pos = robot.readOdometry()
    robot.setSpeed(0, 0)

def prueba_8(robot, r):
    
    girar_odometria(robot, -90, -90)

    desplazamiento(robot, [r*np.pi/4, np.radians(180 / 4)], [40, 0])
    desplazamiento(robot, [r*np.pi/4, np.radians(-180 / 4)], [80, 0])
    desplazamiento(robot, [r*np.pi/4, np.radians(-180 / 4)], [40, 0])
    desplazamiento(robot, [r*np.pi/4, np.radians(180 / 4)], [0, 0])

    girar_odometria(robot, 90, 0)

    # robot.setSpeed(0, np.radians(-90))
    # time.sleep(1)
    # robot.setSpeed(r*np.pi/4, np.radians(180 / 4))
    # time.sleep(4)
    # robot.setSpeed(r*np.pi/4, np.radians(-180 / 4))
    # time.sleep(8)
    # robot.setSpeed(r*np.pi/4, np.radians(180 / 4))
    # time.sleep(4)
    # robot.setSpeed(0, np.radians(90))
    # time.sleep(1)
    # robot.setSpeed(0, 0)


def prueba_2(robot):

    robot.setSpeed(0, np.radians(90))
    time.sleep(1)
    robot.setSpeed(10*np.pi/2, np.radians(-65))
    time.sleep(1)
    robot.setSpeed(22/2, 0)
    time.sleep(2)

    # radio grande
    robot.setSpeed(30*np.pi/4, np.radians(-230 / 4))
    time.sleep(4)

    robot.setSpeed(22/2, 0)
    time.sleep(2)
    robot.setSpeed(10*np.pi/2, np.radians(-65))
    time.sleep(1)
    robot.setSpeed(0, np.radians(-90))
    time.sleep(1)
    robot.setSpeed(0, 0)


def main(args):
    try:
        # Debemos pasarle al main el radio del giro de la trayectoria
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        print("X value at the beginning from main X= %.2f" % (robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        # 2. perform trajectory

        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " %
              (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()
        
        # prueba_8(robot, args.radioD)

        prueba_2(robot)

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()


if __name__ == "__main__":
    # get and parse arguments passed to main
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)
