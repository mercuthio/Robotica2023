from MapLib import Map2D
import numpy as np

map_file = "mapa1.txt"
myMap = Map2D(map_file)
# myMap.drawMap(saveSnapshot=False)
myMap.fillCostMatrix(0, 0, 2, 0)