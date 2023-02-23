import sys
import numpy as np

L = 2 # NO medido
r = 1 # NO medido
th = 0
xWR = [0,0,0]

def setspeed(v,w):
    # v = (vd + vi) / 2
    # w = (vd - vi) / L

    # Se representa el sistema de ecuaciones
    i = np.matrix([[1, 1],[1,-1]])
    d = np.matrix([[2*v],[L*w]])

    vi,vd = (i**-1)*d # Se soluciona el sistema de ecuaciones

    # Aplicar velocidad vd al motor derecho
    # Aplicar velocidad vi al motor izquierdo.

def updateOdometry():
    
    des_sd, des_si = 0 # Asignar valor

    des_s = (des_sd + des_si) / 2
    des_th = (des_sd - des_si) / L

    des_x = des_s * np.cos(th + des_th / 2)
    des_y = des_s * np.sin(th + des_th / 2)

    des_xWR = [des_x, des_y, des_th]
    xWR = xWR + des_xWR
