import matplotlib.pyplot as plt
import plotly.graph_objs as go
import plotly.offline as pyo
from dibrobot import *
from auxFunc import *
import numpy as np
from MapLib import Map2D


def generatePlot_Map(map, f):

    map_file = "maps/mapa" + map + "_CARRERA.txt"
    myMap = Map2D(map_file)

    # Posicion de la salida respecto al mundo
    if map == "A":
        WxS = [600, 2800, np.radians(0)]
        WtS = hom_array(WxS)
    else:
        WxS = [2200, 2800, np.radians(0)]
        WtS = hom_array(WxS)

    with open("logs/" + f, "r") as archivo:
        lineas = archivo.readlines()

    robotPosVectors = []
    for line in lineas:
        x_pos = line.find("X:")
        y_pos = line.find("Y:")
        th_pos = line.find("TH:")
        v_pos = line.find("V:")

        x_val = float(line[x_pos+2:y_pos].strip())
        y_val = float(line[y_pos+2:th_pos].strip())
        th_val = float(line[th_pos+3:v_pos].strip())

        # Posicion del robot respecto a la salida
        SxR = [x_val * 10, y_val * 10, np.radians(th_val)]
        StR = hom_array(SxR)

        WtR = np.dot(WtS, StR)
        WxR = loc_array(WtR)

        robotPosVectors.append(WxR)

    myMap.drawMapWithRobotLocations(robotPosVectors, saveSnapshot=False)


def generatePlot(f):
    with open("logs/" + f, "r") as file:
        fig = go.Figure()
        for i, line in enumerate(file):
            name_ini = line.find("[")
            name_end = line.find("]")
            x_pos = line.find("X:")
            y_pos = line.find("Y:")
            th_pos = line.find("TH:")
            v_pos = line.find("V:")
            w_pos = line.find("W:")

            x_val = line[x_pos+2:y_pos].strip()
            y_val = line[y_pos+2:th_pos].strip()
            th_val = line[th_pos+3:v_pos].strip()
            v_val = line[v_pos+2:w_pos].strip()
            w_val = line[w_pos+2:].strip()
            name_val = line[name_ini+1:name_end].strip()

            fig.add_trace(go.Scatter(x=[float(x_val)], y=[
                float(y_val)], mode='markers', marker=dict(symbol='square', size=30, color='blue', angle=float(th_val)), text="V: " + v_val + " , W: " + w_val + " | TH: " + th_val, name=name_val))

    fig.update_layout(title='Odometría del fichero ' + f,
                      xaxis_title='Eje X',
                      yaxis_title='Eje Y')
    pyo.plot(fig, filename='odometry/odometry-' + f[:-4] + '.html')


def generatePlot_dibrobot(f):
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
