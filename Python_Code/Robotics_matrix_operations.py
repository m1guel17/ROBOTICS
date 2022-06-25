# -*- coding: utf-8 -*-
"""
Created on Sat Jun 18 20:41:20 2022

@author: ESTEBAN
"""
# ======= Libería basada en Robotics Toolbox de Peter Corke para MATLAB =======
import numpy as np
import sympy as sp
from sympy import symbols as syms
dx = sp.symbols("dx"); dy = sp.symbols("dy"); dz = sp.symbols("dz")
x = syms("x"); y = syms("y"); z = syms("z")
# =============================================================================
n = 5 # Decimales de precisión 
# =============================================================================
# Pure Rotation about the x-axis | Rotation about the n-axis | ROLL
def trotx(x):
    if type(x) == sp.Symbol:
        return np.matrix([[1,         0,         0, 0],
                          [0, sp.cos(x),-sp.sin(x), 0],
                          [0, sp.sin(x), sp.cos(x), 0],
                          [0,         0,         0, 1]])
    else:
        x = x*np.pi / 180
        return np.round(np.matrix([[1,         0,         0, 0],
                                   [0, np.cos(x),-np.sin(x), 0],
                                   [0, np.sin(x), np.cos(x), 0],
                                   [0,         0,         0, 1]]),n)
# =============================================================================
# Pure Rotation about the y-axis | Rotation about the o-axis | PITCH
def troty(y):
    if type(y) == sp.Symbol:
        return np.matrix([[sp.cos(y), 0, sp.sin(y), 0],
                          [0,         1,         0, 0],
                          [-sp.sin(y),0, sp.cos(y), 0],
                          [0,         0,         0, 1]])
    else:
        y = y*np.pi / 180
        return np.round(np.matrix([[np.cos(y), 0, np.sin(y), 0],
                                   [0,         1,         0, 0],
                                   [-np.sin(y),0, np.cos(y), 0],
                                   [0,         0,         0, 1]]),n)
# =============================================================================
# Pure Rotation about the z-axis | Rotation about the a-axis | YAW
def trotz(z):
    if type(z) == sp.Symbol:
        return np.matrix([[sp.cos(z),-sp.sin(z), 0, 0],
                          [sp.sin(z), sp.cos(z), 0, 0],
                          [0,         0,         1, 0],
                          [0,         0,         0, 1]])
# =============================================================================
#       return np.matrix([[sp.cos(z),-sp.sin(z),"   0","   0"],
#                         [sp.sin(z), sp.cos(z),"    0","   0"],
#                         ["   0","    0","   1","   0"],
#                         ["   0","    0","   0","   1"]])
# =============================================================================
    else:
        z = z*np.pi / 180
        return np.round(np.matrix([[np.cos(z),-np.sin(z), 0, 0],
                                   [np.sin(z), np.cos(z), 0, 0],
                                   [0,         0,         1, 0],
                                   [0,         0,         0, 1]]),n)
# =============================================================================
# Composed Rotations | trotx(x)@troty(y)@trotz(z)
def trotxyz(x,y,z):
    return trotx(x)*troty(y)*trotz(z)
# =============================================================================
# Pure Translation along the xyz axes | Pure Translation along the noa axes
def transl(x,y,z):
    if type(x) == sp.Symbol or type(y) == sp.Symbol or type(z) == sp.Symbol:
        return np.matrix([[1, 0, 0, x],
                          [0, 1, 0, y],
                          [0, 0, 1, z],
                          [0, 0, 0, 1]])
    else:
        return np.round(np.matrix([[1, 0, 0, x],
                                   [0, 1, 0, y],
                                   [0, 0, 1, z],
                                   [0, 0, 0, 1]]),2)


Fnoa = np.matrix([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])


