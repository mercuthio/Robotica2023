import sys
from dibrobot import *
from plotManager import generatePlot
from plotManager import generatePlot_Map

if len(sys.argv) == 2:
    f = sys.argv[1]
    generatePlot(f)
elif len(sys.argv) == 3:
    f = sys.argv[1]
    map = sys.argv[2]
    generatePlot_Map(map, f)
else:
    print("Usage: python3 plotOdometry.py [fileName] <mapID {A | B}>")
