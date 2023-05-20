import cv2
from cv2 import sqrt
import numpy as np
import os
import matplotlib.pyplot as plt
import statistics
from mpl_toolkits.mplot3d import Axes3D


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


def force(v):
    return -k*v

def acceleration(v):
    return np.array([0,-g,0]) # ToDo add drag coefficient

g = 9.81 # acceleration due to gravity (m/s^2)
# ToDO k = 0.0   # drag coefficient
dt = 1/250 # time step
t = 0 # Not using. t = equal to the time of the fall
v = np.array([5.5, 1.28*g, 0.0])  # initial velocity m/s
x0 =   -7.0
y0 = -4.0
z0 =  25.0
r = np.array([x0, y0, z0])  
countOfValues = round((1/dt)*2.0*v[1]/g)-1

for i in range (countOfValues):
    a = acceleration(v)
    v = v + a*dt
    r = r + v*dt
    t += dt
    print ("Position: ", r)

















DeleteTrash(r"./BallDetectorLogSobelVectorDotVideo63WithOffset1QMore.txt") 
FindCord63SobelOffset = np.loadtxt(r"./BallDetectorLogSobelVectorDotVideo63WithOffset1QMore.txt")
BlenderCord63SobelOffset  = np.loadtxt(r"./LogBlenderCoord63video.txt", delimiter=',')

# Define the data points
x_blender = BlenderCord63SobelOffset[:,0] # x-coordinates of the points
y_blender = BlenderCord63SobelOffset[:,1] # y-coordinates of the points
z_blender = BlenderCord63SobelOffset[:,2] # z-coordinates of the points

x_find = FindCord63SobelOffset[:,0] # x-coordinates of the points
y_find = FindCord63SobelOffset[:,1] # y-coordinates of the points
z_find = FindCord63SobelOffset[:,2] # z-coordinates of the points
# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_blender, y_blender, z_blender, '-', label = "True trajectory")

ax.plot(x_find, y_find, z_find, "-s", label = "Finding points")
# Set the axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Set the x and y limits
plt.axis('equal')

plt.legend()
# Rotate the plot
#ax.view_init(elev=90, azim=-90)
# Show the plot
plt.show()