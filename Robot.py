#!/usr/bin/python
# -*- coding: UTF-8 -*-
# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division  # ''

# import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import numpy as np
import datetime
import brickpi3
import get_blob
import cv2

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        self.R = 2.6
        self.L = 11.8

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        # self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.BP.PORT_B,
                                     self.BP.get_motor_encoder(self.BP.PORT_B))
        self.BP.offset_motor_encoder(self.BP.PORT_C,
                                     self.BP.get_motor_encoder(self.BP.PORT_C))

        ##################################################
        # odometry shared memory values
        self.x = Value('d', 0.0)
        self.y = Value('d', 0.0)
        self.th = Value('d', 0.0)
        self.v = Value('d', 0.0)
        self.w = Value('d', 0.0)
        # boolean to show if odometry updates are finished
        self.finished = Value('b', 1)

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        self.P = 0.01
        self.log_file = open(datetime.datetime.now().strftime(
            "logs/log-%Hh-%Mm-%Ss.txt"), "w")

    def setSpeed(self, v, w):
        """Speed v and w is applied to both engines"""

        print("setting speed to %.2f %.2f" % (v, w))

        speedDPS_left = np.degrees((2 * v - self.L * w) / (2 * self.R))
        speedDPS_right = np.degrees((2 * v + self.L * w) / (2 * self.R))

        # print("Velocidad: {}, {}".format(speedDPS_left, speedDPS_right))

        self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_left)
        self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_right)

        # self.log_file.write("Actualizada velocidad = WI:{},WD:{}".format(speedDPS_left, speedDPS_right))

    def readSpeed(self):
        """Returns the v and w from the engines"""

        self.lock_odometry.acquire()

        v = self.v.value
        w = self.w.value

        self.lock_odometry.release()
        return v, w

    def readOdometry(self):
        """Returns the position of the robot"""
        return self.x.value, self.y.value, np.degrees(self.th.value)

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        # additional_params?))
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self):  # , additional_params?):
        """Updates the position of the robot every self.P seconds"""
        encoder_C = 0
        encoder_B = 0

        while not self.finished.value:
            # obtenemos el tiempo actual
            tIni = time.clock()

            # Obtenemos la velocidad de los encoders de los motores en la diferencia de tiempo.
            diferencia_C = (self.BP.get_motor_encoder(
                self.BP.PORT_C) - encoder_C) / self.P
            diferencia_B = (self.BP.get_motor_encoder(
                self.BP.PORT_B) - encoder_B) / self.P

            encoder_C = self.BP.get_motor_encoder(self.BP.PORT_C)
            encoder_B = self.BP.get_motor_encoder(self.BP.PORT_B)

            self.v.value = (
                self.R * (np.radians(diferencia_C) + np.radians(diferencia_B))) / 2
            self.w.value = (self.R * (np.radians(diferencia_C) -
                            np.radians(diferencia_B))) / self.L

            As = self.v.value * self.P
            Ath = self.w.value * self.P

            Ax = As * np.cos(self.th.value + Ath / 2)
            Ay = As * np.sin(self.th.value + Ath / 2)

            ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
            # sys.stdout.write("Dummy update of odometry ...., X=  %d, \
            #     Y=  %d, th=  %d \n" %(self.x.value, self.y.value, self.th.value) )
            # print("Dummy update of odometry ...., X=  %.2f" %(self.x.value) )

            # Bloqueamos mutex de la odometría.
            self.lock_odometry.acquire()

            # Actualizamos los valores que tenemos de la odometría según su incremento.
            self.x.value += Ax
            self.y.value += Ay
            self.th.value += Ath

            # Guardamos cómo se ha modificado la odometría en un log y lo imprimimos por pantalla

            # Desbloqueamos mutex de la odometría.
            self.lock_odometry.release()

            # save LOG
            # print("[{}] Actualizada posición = X:{}\tY:{}\tTH:{}\tV:{}\tW:{}\n".format(datetime.datetime.now().strftime("%Hh-%Mm-%Ss.txt"),
                #   round(self.x.value, 2), round(self.y.value, 2), round(np.degrees(self.th.value), 2), round(self.v.value, 2), round(self.w.value, 2)))
            self.log_file.write("[{}] Actualizada posición = X:{}\tY:{}\tTH:{}\tV:{}\tW:{}\n".format(datetime.datetime.now().strftime(
                "%Hh-%Mm-%Ss"), round(self.x.value, 2), round(self.y.value, 2), round(np.degrees(self.th.value), 2), round(self.v.value, 2), round(self.w.value, 2)))
            self.log_file.flush()

            # try:
            # Each of the following BP.get_motor_encoder functions returns the encoder value
            # (what we want to store).
            # sys.stdout.write("Reading encoder values .... \n")
            # [encoder1, encoder2] = [self.BP.get_motor_encoder(self.BP.PORT_B),
            #    self.BP.get_motor_encoder(self.BP.PORT_C)]
            # except IOError as error:
            # print(error)
            # sys.stdout.write(error)

            # sys.stdout.write("Encoder (%s) increased (in degrees) B: %6d  C: %6d " %
            #        (type(encoder1), encoder1, encoder2))

            ######## UPDATE UNTIL HERE with your code ########

            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))

        # print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" % (self.x.value, self.y.value, self.th.value))

    # Stop the odometry thread.

    def stopOdometry(self):
        """Kills the process that updates odometry"""
        self.finished.value = True
        self.log_file.close()
        # self.BP.reset_all()

    # Operación para perseguir y capturar la pelota.
    def trackObject(self, targetSize, target, colorRangeMin=[0,0,0], colorRangeMax=[255,255,255]):

        finished = False
        targetFound = False
        targetPositionReached = False

        # A es el área que debe tener la pelota cuando el robot la tenga delante
        # para cogerla
        A = np.pi * (targetSize / 2)**2 
        x_anterior = 0

        while not finished:
            # 1. search the most promising blob ..
            while not targetFound:
                # Dar vueltras buscando la pelota
                blob = get_blob.get_blob()
                print(blob)

                # Si ha encontrado la pelota, sale del bucle
                if (blob != -1):
                    self.setSpeed(0, 0)
                    targetFound = True
                    break
                
                # Si no ha encontrado la pelota, da vueltas
                if (x_anterior < target):
                    self.setSpeed(0, np.radians(60))
                else:
                    self.setSpeed(0, np.radians(-60))

            while not targetPositionReached: 
                # 2. decide v and w for the robot to get closer to target position
                print(blob[0])
                x_anterior = blob[0]

                # Revisa si sigue teniendo la pelota delante, si no la tiene
                # volvemos a buscarla
                blob = get_blob.get_blob()
                if (blob == -1):
                    targetFound = False
                    break
                
                # a es el área de la pelota que ha encontrado y d la distancia
                # de la pelota en la imágen de donde debería estar.
                # blob[1] = diametro, blob[0] = x
                a = np.pi * (blob[1] / 2)**2
                d = blob[0] - target 
                print(a)
                print(d)
                # Sacamos una velocidad lineal y angular en función de la 
                # distancia y el área de la pelota para perseguirla.
                v = np.clip(A-a, 0, 20)
                w = np.radians(np.clip(-d, -20, 20))
                self.setSpeed(v, w)

                # Cuando la diferencia de área y distancia es suficientemente
                # pequeña, paramos y cogemos la pelota
                if A-a <= 3 and np.abs(d) <= 0.1: # Medir valores con pelota en posicion correcta 
                    targetPositionReached = True
                    finished = True
                    self.setSpeed(20,0)
                    time.sleep(1)
                    self.setSpeed(0,0)
                    self.catch()

    def catch(self):
        # Bajar cesta
        speed = 150
        self.BP.set_motor_dps(self.BP.PORT_A, speed)
        time.sleep(0.5)
        self.BP.set_motor_dps(self.BP.PORT_A, 0)

    def uncatch(self):
        # Subir cesta
        speed = -150
        self.BP.set_motor_dps(self.BP.PORT_A, speed)
        time.sleep(0.5)
        self.BP.set_motor_dps(self.BP.PORT_A, 0)

