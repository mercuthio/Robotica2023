import sys
import numpy as np
import matplotlib.pyplot as plt
from dibrobot import *


def generatePlot(f):
    with open("logs/" + f, "r") as file:
        for line in file:
            x_pos = line.find("X:")
            y_pos = line.find("Y:")
            th_pos = line.find("TH:")
            v_pos = line.find("V:")

            x_val = line[x_pos+2:y_pos].strip()
            y_val = line[y_pos+2:th_pos].strip()
            th_val = line[th_pos+3:v_pos].strip()
            dibrobot([float(x_val), float(y_val),
                      float(th_val)], 'purple', 'p')
        plt.show()


if len(sys.argv) == 2:
    f = sys.argv[1]
    generatePlot(f)
else:
    print("Usage: python3 plotOdometry.py [fileName]")
