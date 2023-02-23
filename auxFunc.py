import numpy as np
import math

def norm_pi(ang):
    while ang > math.pi:
        ang -= 2*math.pi
    while ang < -math.pi:
        ang += 2*math.pi
    return ang

def loc_array(t):
    x = np.array([t[0,2], t[1,2], np.arctan2(t[1,0],t[0,0])])
    return x

def hom_array(vector):
    x, y, th_rad = vector    # x es un vector de tres componentes
    mat = np.array([ [np.cos(th_rad), -np.sin(th_rad), x],
                     [np.sin(th_rad),  np.cos(th_rad), y],
                     [0, 0, 1]] )
    return mat

def hom(vector):
    th_rad = vector.th
    mat = np.array([ [np.cos(th_rad), -np.sin(th_rad), vector.dx],
                     [np.sin(th_rad),  np.cos(th_rad), vector.dy],
                     [0, 0, 1]] )
    return mat
