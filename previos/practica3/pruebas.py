from MapLib import Map2D
import numpy as np

map_file = "mapa1.txt"
myMap = Map2D(map_file)
myMap.fillCostMatrix(0, 0, 2, 0)
myMap.planPath(0, 0, 2, 0)
print(myMap.costMatrix)
print(myMap.currentPath)
myMap.drawMap(saveSnapshot=False)