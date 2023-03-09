import math
import time
import numpy as np

def trackObject(self, targetSize, target, colorRangeMin=[0,0,0], colorRangeMax=[255,255,255]):
    # , catch=??, ...)

    finished = False
    targetFound = False
    targetPositionReached = False

    A = math.pi * (targetSize / 2)**2 

    while not finished:
        # 1. search the most promising blob ..

        while not targetFound:
            # Dar vueltras buscando la pelota
            blob = self.get_blob(colorRangeMin, colorRangeMax)
            if (blob != -1):
                targetFound = True
                break
            self.set_speed(0, np.radians(10))

        while not targetPositionReached: 
            # 2. decide v and w for the robot to get closer to target position

            blob = self.get_blob(colorRangeMin, colorRangeMax)
            if (blob == -1):
                targetFound = False
                break

            # blob.size = diametro
            a = math.pi * (blob.size / 2)**2
            d = blob.x - target 

            v = A-a
            w = -d
            self.set_speed(v, w)

            if A-a <= 3 and d <= 0.1: # Medir valores con pelota en posicion correcta 
                targetPositionReached = True
                finished = True

def catch(self):
    # decide the strategy to catch the ball once you have reached the target position

    # Bajar cesta
    speed = 5
    self.BP.set_motor_dps(self.BP.PORT_A, speed)
    time.sleep(3)
    self.BP.set_motor_dps(self.BP.PORT_A, 0)