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
from get_blob import get_blob
from get_blob import get_red
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

        # Diccionario con las posibles acciones del robot, (orientacion del robot, posicion objetivo)
        # El robot está mirando hacia [...] y su próximo movimiento está hacia [...], así que tiene que hacer [...]
        self.acciones = {
            ("Norte", "Oeste"): 90,
            ("Norte", "Este"): -90,
            ("Norte", "Sur"): 180,

            ("Sur", "Oeste"): -90,
            ("Sur", "Este"): 90,
            ("Sur", "Norte"): 180,

            ("Este", "Norte"): 90,
            ("Este", "Sur"): -90,
            ("Este", "Oeste"): 180,

            ("Oeste", "Norte"): -90,
            ("Oeste", "Sur"): 90,
            ("Oeste", "Este"): 180
        }

    def setSpeed(self, v, w):
        """Speed v and w is applied to both engines"""

        # print("setting speed to %.2f %.2f" % (v, w))

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
        self.p = Process(target=self.updateOdometry, args=())
        self.p.start()

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
            self.log_file.write("[{}] X:{}\tY:{}\tTH:{}\tV:{}\tW:{}\n".format(datetime.datetime.now().strftime(
                "%Hh-%Mm-%Ss"), round(self.x.value, 2), round(self.y.value, 2), round(np.degrees(self.th.value), 2), round(self.v.value, 2), round(self.w.value, 2)))
            self.log_file.flush()

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
    def trackObject(self, targetSize, target, colorRangeMin=[0, 0, 0], colorRangeMax=[255, 255, 255]):
        """Tracks the object until the robot can catch it"""

        finished = False
        targetFound = False
        targetPositionReached = False

        # A es el área que debe tener la pelota cuando el robot la tenga delante
        # para cogerla
        A = targetSize
        x_anterior = 0

        old_a = 0

        # Mientras no termine el ejercicio de buscar la pelota
        while not finished:

            # 1 - Busca la pelota girando sobre sí mismo
            while not targetFound:

                # Da vueltras buscando la pelota
                blob = get_blob(False)

                # Si ha encontrado la pelota, sale del bucle
                if (blob != -1):
                    targetFound = True
                    targetPositionReached = False
                    old_a = blob[1]
                    break

                # Si no ha encontrado la pelota da vueltas
                if (x_anterior < target):
                    self.setSpeed(0, np.radians(60))
                else:
                    self.setSpeed(0, np.radians(-60))

            # 2 - Decido un v y un w para acercarme a la pelota
            while not targetPositionReached:

                x_anterior = blob[0]

                # a es el área de la pelota que ha encontrado y d la distancia
                # de la pelota en la imágen de donde debería estar.
                # blob[1] = diametro, blob[0] = x
                a = blob[1]  # type: ignore
                d = blob[0] - target
                # Sacamos una velocidad lineal y angular en función de la
                # distancia y el área de la pelota para perseguirla.

                # Miramos si el getblob se ha rallado y ha dado un tiron
                if np.abs(old_a - a) > 30:
                    a = old_a
                    v = -10
                    w = 0
                    print("Posición errónea, recalculando...")
                    print("DIF. AREA:", A-a, "| D:", d, "| v, w:", v, w)
                    self.setSpeed(v, w)
                    targetFound = False
                    targetPositionReached = False
                    break
                else:
                    old_a = a
                    v = np.clip((A-a) / 10, -10, 30)
                    w = np.radians(np.clip(-d / 10, -20, 20))

                self.setSpeed(v, w)

                print("DIF. AREA:", A-a, "| D:", d, "| v, w:", v, w)

                # Cuando la diferencia de área y distancia es suficientemente
                # pequeña, paramos y cogemos la pelota
                # Medir valores con pelota en posicion correcta

                MARGEN_AREA = 50
                MARGEN_DISTANCIA = 30  # 3

                # Comprobamos si tenemos ya la pelota delante nuestro
                if (A-a) <= MARGEN_AREA and np.abs(d) <= MARGEN_DISTANCIA:

                    # Si sigo viendo la pelota
                    if targetFound:
                        targetPositionReached = True

                        # Avanzo hasta la pelota
                        v_fin = 5.5

                        if (A-a) <= 0:
                            v_fin += (A-a) / 10000.0

                        print("Seteando velocidad final", v_fin)
                        self.setSpeed(v_fin, 0)
                        time.sleep(2)
                        self.setSpeed(0, 0)

                        # Bajo la cesta
                        self.catch()

                        # Espero un tiempo para comprobar
                        time.sleep(1)

                        # Se comprueba si se ha obtenido la pelota

                        blob_red = get_red(False)
                        blob_red = get_red(False)

                        if blob_red:
                            self.setSpeed(0, np.radians(-90))
                            time.sleep(1)
                            self.setSpeed(0, 0)

                            blob_red = get_red(False)
                            blob_red = get_red(False)

                            # Si ve bastante rojo, la ha cogido
                            if blob_red:
                                # Pelota conseguida
                                finished = True
                                print("Finalizado con éxito! Soltando pelota.")
                            # No ha atrapado la pelota
                            else:
                                print("No he conseguido atrapar la pelota.")

                                targetPositionReached = True
                                targetFound = False
                                # Marcha atrás para mejorar visión
                                blob = get_blob(False)

                            self.setSpeed(0, np.radians(90))
                            time.sleep(1)
                            self.setSpeed(0, 0)
                            time.sleep(3)
                            self.uncatch()
                            break

                # Revisa si sigue teniendo la pelota delante, si no la tiene
                # volvemos a buscarla
                blob = get_blob(False)
                if (blob == -1):
                    targetFound = False
                    break

    # Bajar la cesta
    def catch(self):
        """Moves down the basket"""
        print("Bajando cesta")
        speed = 140
        self.BP.set_motor_dps(self.BP.PORT_A, speed)
        time.sleep(0.6)
        self.BP.set_motor_dps(self.BP.PORT_A, 0)

    # Subir la cesta
    def uncatch(self):
        """Moves up the basket"""
        print("Subiendo cesta")
        speed = -140
        self.BP.set_motor_dps(self.BP.PORT_A, speed)
        time.sleep(0.6)
        self.BP.set_motor_dps(self.BP.PORT_A, 0)

    # Mueve al robot de la posicion ini a la posición next
    def goTo(self, x_ini, y_ini, x_next, y_next):

        # Sacamos la orientación de a dónde se tiene que mover el robot
        if (x_ini == x_next):
            if (y_ini < y_next):
                orientacion_destino = "Norte"
            else:
                orientacion_destino = "Sur"
        else:
            if (x_ini < x_next):
                orientacion_destino = "Este"
            else:
                orientacion_destino = "Oeste"

        # Calculamos la orientación del robot
        _, _, grados = self.readOdometry()

        if grados < 0:  # Si son negativos
            grados = grados % 360
            grados = 360 - grados
        else:
            grados = grados % 360

        if grados < 45 or grados > 315:
            orientacion_robot = "Norte"
        elif grados < 135:
            orientacion_robot = "Oeste"
        elif grados < 225:
            orientacion_robot = "Sur"
        else:  # grados < 315
            orientacion_robot = "Este"

        # Si acciones contiene el elemento...
        if (orientacion_robot, orientacion_destino) in self.acciones:
            self.setSpeed(0, np.radians(
                self.acciones[(orientacion_robot, orientacion_destino) / 2]))
            time.sleep(2)

        # Movemos hacia deltante
        self.setspeed(40 / 2, 0)
        time.sleep(2)
