#!/usr/bin/env python
# coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
from ipywidgets import *
from IPython.display import display

def DeleteTrash(pathTxtFile):
    with open (pathTxtFile, 'r') as f:
        old_data = f.read()
        new_data = old_data.replace('[', '')
        new_data = new_data.replace(']', '')
        new_data = new_data.replace(',', '')
        new_data = new_data.replace(';', '')
        new_data = new_data.replace('(', '')
        new_data = new_data.replace(')', '')
    with open (pathTxtFile, 'w') as f:
        f.write(new_data)





FindCord = np.loadtxt(r"./BallDetectorLogMomentWithCameraMatrix.txt")
BlenderCord63 = np.loadtxt(r"./LogBlenderCoord63video.txt", delimiter=',')
dt = 1.0/25.0


def paint(TrueCord, FindCord):
    fig, ax = plt.subplots()
    ax.plot(FindCord, label = "Find")
    ax.plot(TrueCord, label = "True")
    ax.grid()
    ax.legend()
    #print (TrueCord, " \n vs \n", FindCord)
    #plt.show()

vx_True = np.diff(BlenderCord63[:,0]) /dt
vx_Find = np.diff(FindCord[:,0]) /dt

print("velocity: True", vx_True , '\n vs Find \n' , vx_Find )
Loc = interact(paint(BlenderCord63[:,0], FindCord[:,0]) ) # Loc
Velocity = interact(paint(vx_True, vx_Find ) ) #Velocity




