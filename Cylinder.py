'''import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Радиус цилиндра и его высота
r = 1
h = 2

# Генерация данных для цилиндра
theta = np.linspace(0, 2*np.pi, 100)
z = np.linspace(0, h, 50)
theta, z = np.meshgrid(theta, z)
x = r * np.cos(theta) * np.tan(45)
y = r * np.sin(theta)* np.tan(45)

# Создание 3D-объекта цилиндра и отображение его
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=30, azim=45)
ax.plot_surface(x, y, z, alpha=0.7)
plt.show()
'''

import cv2

# Load image
img = cv2.imread('video48Trajectory.png')

# Apply Gaussian blur
img_blur = cv2.GaussianBlur(img, (5,5), 0)

# Show image with and without blur for comparison
cv2.imshow('Original image', img)
cv2.imshow('Blurred image', img_blur)
cv2.waitKey(0)