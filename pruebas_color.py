import argparse
import cv2
import numpy as np
import time
from Robot import Robot
from MapLib import Map2D

robot = Robot()
for i in range(0, 20):
    print(robot.read_luminosity())
    time.sleep(1)