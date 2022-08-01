from distutils import core
import cv2
import numpy as np
import os
import glob
import json

def undistortionImage():
    images = glob.glob('clear.bmp')
    f = open('intrinsics0.json')
    data = json.load(f)['intrinsics']
    K = np.array(data['K']).reshape(3,-1) # camera matrix
    dist = data['distortion'] # distortion coefficients
    i = 0
    for fname in images:
        i += 1
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        dist= np.asarray(dist) 
        undistorted_image = cv2.undistort(img, K, dist, None, K)
        cv2.imwrite("clear.png", undistorted_image)
        #cv2.imwrite("Undistort"+str(i)+".png", undistorted_image)
def calibrateMy():
    img = cv2.imread("D:\\Sirius\\BallDetector\\dataFromRealCamera\\undisturtImages\\White5.png")
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    CHECKERBOARD = (3,4)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1000, 0.01)
    # Найти углы шахматной доски
    # Если на изображении найдено нужное количество углов, тогда ret = true
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    """
    Если желаемый номер угла обнаружен,
    уточняем координаты пикселей и отображаем
    их на изображениях шахматной доски
    """
    # Создание вектора для хранения векторов трехмерных точек для каждого изображения шахматной доски
    objpoints = []
    # Создание вектора для хранения векторов 2D точек для каждого изображения шахматной доски
    imgpoints = [] 
    # Определение мировых координат для 3D точек
    a = 0.0445
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * a
    prev_img_shape = None
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
    #cv2.imshow("s", img)
    print ("rvecs: ",rvecs, "  tvec:  ", tvecs)
    print ("corners2: ", corners2[0])
    cv2.waitKey(0)
undistortionImage()