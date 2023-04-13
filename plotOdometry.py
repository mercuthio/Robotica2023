import sys
from dibrobot import *
from plotManager import generatePlot

if len(sys.argv) == 2:
    f = sys.argv[1]
    generatePlot(f)
else:
    print("Usage: python3 plotOdometry.py [fileName]")
