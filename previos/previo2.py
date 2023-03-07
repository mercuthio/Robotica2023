import sys
import numpy as np
from simubot import *
from dibrobot import *


L = 2 # NO medido
r = 1 # NO medido
t = 0.1
wi = 0
wd = 0
xWR = [0,0,0]

def setspeed(v,w):
    wi = (2 * v - L * w) / (2 * r)
    wd = (2 * v + L * w) / (2 * r)

    # Aplicar velocidad vd al motor derecho
    # BP.setdps(A, wi)
    # Aplicar velocidad vi al motor izquierdo.
    # BP.setdps(B, wd)

def readSpeed():
    # Obtener mtx
    v = r * (wd + wi) / 2
    w = r * (wd - wi) / L
    # Devolver mtx
    return v, w

def updateOdometry():
    v, w = readSpeed()

    As = v * t
    Ath = w * t

    Ax = As * np.cos(xWR.th + Ath / 2)
    Ay = As * np.sin(xWR.th + Ath / 2)

    des_xWR = [Ax, Ay, Ath]
    xWR = xWR + des_xWR

# Calculamos primero R sustituyendo los valores de xRP
xi = [0,0,0]    
xf = [0,4,math.radians(180)]
R = (xf[0]**2 + xf[1]**2) / (2 * xf[1])
t = 1

dist = 2 * math.pi
v = dist / t
w = v / R
vc = np.array([v, w])
print("     v:{}, w:{}".format(round(v,2), round(w,2)))

xRP_aux = [0,0,0] # posicion inicial del robot
delay = 0.01

plt.plot([-2,0], [0,0], 'black')
plt.plot([0,2], [0,0], 'black')
plt.plot([-2,2], [8,8], 'black')
plt.plot([-2,-2], [0,8], 'black')
plt.plot([2,2], [0,8], 'black')

entrado = False

for i in range(int((1/delay) * t * 3)):
    xRP_aux = simubot(vc,xRP_aux,delay)
    dibrobot(xRP_aux,"blue","p")
    plt.pause(delay)

plt.show()