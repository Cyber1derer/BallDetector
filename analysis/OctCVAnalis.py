
from tkinter.messagebox import NO
import cv2
from cv2 import sqrt
import numpy as np
import os
import json
import matplotlib.pyplot as plt
import statistics

from pip import main

def Otnos_path():
    dirname = os.path.dirname(__file__)
    path = "C:/Users/Student/DenisV/kurs/Oct2/BallDetector/analysis/BallDetectorLog.txt"
    start = dirname
    relative_path = os.path.relpath(path, start)
    print(relative_path)

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

DeleteTrash("C:/Users/Student/DenisV/kurs/Oct2/BallDetector/analysis/BallDetectorLog3DMove50.txt")
FindCord = np.loadtxt("C:/Users/Student/DenisV/kurs/Oct2/BallDetector/analysis/BallDetectorLog3DMove50.txt")

DeleteTrash("C:/Users/Student/DenisV/kurs/Oct2/BallDetector/analysis/LogBlenderCoord3DMove50.txt")
TrueCord = np.loadtxt("C:/Users/Student/DenisV/kurs/Oct2/BallDetector/analysis/LogBlenderCoord3DMove50.txt")

with open('C:/Users/Student/DenisV/kurs/Oct2/BallDetector/CameraParametrs.json') as f:
    templates = json.load(f)

def myNorm(a=np.array([1,2,3]),b=np.array([2,2,4])):
    return np.linalg.norm(a - b, ord=2)

rVecR = np.array(templates["extrinsics"]["rvecR"])
tVec = np.array ( templates["extrinsics"]["Tvec"]  ) 

distortion = np.array(templates["intrinsics"]["distortion"])
CameraMatrix = np.array(templates["intrinsics"]["K"])
CameraMatrix = CameraMatrix.reshape((3,3))

TrueCord[:,2] = TrueCord[:,2] -  0.05
ProjectPointsFind, jacobFind = cv2.projectPoints(FindCord, rVecR, tVec,CameraMatrix, distortion)
ProjectPointsTrue, jacobTrue = cv2.projectPoints(TrueCord, rVecR, tVec,CameraMatrix, distortion)

#plt.scatter(ProjectPointsFind[:,0,0] , ProjectPointsFind[:,0,1] )
#plt.scatter(ProjectPointsTrue[:,0,0] , ProjectPointsTrue[:,0,1] )

#Norm =  (pow( (ProjectPointsFind[:,0,0] - ProjectPointsTrue[:,0,0]), 2) + pow( (ProjectPointsFind[:,0,1] - ProjectPointsTrue[:,0,1]), 2))
NormX = ProjectPointsFind[:,0,0] - ProjectPointsTrue[:,0,0]
NormY = ProjectPointsFind[:,0,1] - ProjectPointsTrue[:,0,1]
print ("Statistics mean:  ", statistics.mean(NormX), " error in px " ) 
Sigma = statistics.stdev(NormX)
print ("Statistics stdev:  ", Sigma, " error in px" )
print (NormX)




plt.hist(NormX, bins = 20, alpha = 0.75, color = 'blue', label = 'Norm2')
#print(  ProjectPointsFind - ProjectPointsTrue)
plt.show()
cv2.waitKey(0)

if __name__=="__main__":
    Otnos_path()




















def ThisIsTrash():
    # Проверка корректности стандартного отклонения
    #data = np.random.normal(loc=0.0, scale=2.0, size=100000)
    #print ("Data:  ", data)
    #print ("np.std(data):   ", np.std(data)) 
    #print ("dispersion:   ", np.var(data) )
    #res_std = statistics.stdev(data)
    #print ("statistics.stdev(data):   ", res_std) 
    print ("Norm:   ", Norm)
    print ("Statistics mean:  ", statistics.mean(Norm) ) 
    print ("Statistics stdev:  ",statistics.stdev(Norm) )
    print ("Statistics pstdev:  ",statistics.pstdev(Norm) )
    print ("Np mean", np.mean(Norm))
    print ("Np std", np.std(Norm))