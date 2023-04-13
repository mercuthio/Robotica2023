import math


def convertir_rango(angulo):
    angulo = (angulo + 180) % 360 - 180
    return angulo


print(convertir_rango(-180))
