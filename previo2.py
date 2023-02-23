import sys
import numpy as np

L = 2 # NO medido
r = 1 # NO medido
th = 0
xWR = [0,0,0]

def setspeed(v,w):
    # v = (vd + vi) / 2
    # w = (vd - vi) / L

    vi = 2 * v - vd
    vd = w * L + (2 * v - vd)

    vi = 2 * v - vd

    # Aplicar velocidad vd al motor derecho
    # Aplicar velocidad vi al motor izquierdo

def updateOdometry():
    # Obtener des_s, des_delta
    des_s = 0
    des_th = 0

    des_x = des_s * np.cos(th + des_th / 2)
    des_y = des_s * np.sin(th + des_th / 2)

    des_xWR = [des_x, des_y, des_th]
    xWR = xWR + des_xWR
