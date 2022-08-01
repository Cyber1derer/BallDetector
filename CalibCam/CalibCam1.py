#!/usr/bin/env python
import cv2
import numpy as np
import os
import glob
# Определение размеров шахматной доски
CHECKERBOARD = (3,4)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1000, 0.01)
# Создание вектора для хранения векторов трехмерных точек для каждого изображения шахматной доски
objpoints = []
# Создание вектора для хранения векторов 2D точек для каждого изображения шахматной доски
imgpoints = [] 
# Определение мировых координат для 3D точек
a = 1
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * a
prev_img_shape = None
# Извлечение пути отдельного изображения, хранящегося в данном каталоге
images = glob.glob('D:/Sirius/BallDetector/dataFromRealCamera/test/*.png')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Найти углы шахматной доски
    # Если на изображении найдено нужное количество углов, тогда ret = true
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    
    """
    Если желаемый номер угла обнаружен,
    уточняем координаты пикселей и отображаем
    их на изображениях шахматной доски
    """
    if ret == True:
        objpoints.append(objp)
        # уточнение координат пикселей для заданных 2d точек.
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
        
        imgpoints.append(corners2)
        # Нарисовать и отобразить углы
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
    
    cv2.imshow('img',img)
    cv2.waitKey(0)
cv2.destroyAllWindows()
h,w = img.shape[:2]
"""
Выполнение калибровки камеры с помощью
Передача значения известных трехмерных точек (объектов)
и соответствующие пиксельные координаты
обнаруженные углы (imgpoints)
"""
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
Rvecs = []
for i in range(len(tvecs)):
    Rvec = np.zeros((3,3))
    cv2.Rodrigues(rvecs[i], Rvec)
    Rvecs.append(Rvec)

point_world = np.array([[[0], [0], [0]], [[0], [1], [0]], [[0], [1], [0]], [[3], [4], [0]]])
point_img = []
point_world_new = []
for i in range(4):
    p_img = mtx @ (Rvecs[0] @ point_world[i] + tvecs[0])
    point_img.append(p_img)
    point_world_new.append(Rvecs[0].T @ (np.linalg.inv(mtx) @ p_img - tvecs[0]))

print("Camera matrix : \n")
print(mtx)
print("dist : \n")
print(dist)
print("rvecs : \n")
print(rvecs)
print("tvecs : \n")
print(tvecs)
print("Rvecs : \n")  
print(Rvecs)
print("P : \n")  
print(point_img)
print(point_world_new)
print("objp : \n")
print(objp)


file = open("Calib_Param.txt", 'w')
mtxinv = np.linalg.inv(mtx)
for strn in mtxinv:
    file.write(str(strn[0]) + '\n' + str(strn[1]) + '\n' + str(strn[2]) + '\n')
    
RvecsT = Rvecs[0].T
for strn in RvecsT:
    file.write(str(strn[0]) + '\n' + str(strn[1]) + '\n' + str(strn[2]) + '\n')

file.write(str(tvecs[0][0, 0]) + '\n' + str(tvecs[0][1, 0]) + '\n' + str(tvecs[0][2, 0]) + '\n')
file.close()





















