import numpy as np

from MapLib import Map2D
from Robot import Robot

# Inicializo el robot
robot = Robot()

map_file = "maps/mapa1.txt"
myMap = Map2D(map_file)

# myMap.drawMap(saveSnapshot=False)

myMap.go(robot, 2, 0)

# myMap.fillCostMatrix(0, 0, 2, 0)
# myMap.planPath(0, 0, 2, 0)
# print(myMap.costMatrix)
# print(myMap.currentPath)
# myMap.drawMap(saveSnapshot=False)
