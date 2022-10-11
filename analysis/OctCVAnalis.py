
import cv2
import numpy as np
import os
import json
import matplotlib.pyplot as plt

def Otnos_path():
    dirname = os.path.dirname(__file__)
    path = "D:/Sirius/FolderForBallDetector/OctBall/BallDetector/analysis/BallDetectorLog.txt"
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

DeleteTrash("D:/Sirius/FolderForBallDetector/OctBall/BallDetector/analysis/BallDetectorLog3DMove10.txt")
FindCord = np.loadtxt("D:/Sirius/FolderForBallDetector/OctBall/BallDetector/analysis/BallDetectorLog3DMove10.txt")

DeleteTrash("D:/Sirius/FolderForBallDetector/OctBall/BallDetector/analysis/LogBlenderCoord3DMove10.txt")
TrueCord = np.loadtxt("D:/Sirius/FolderForBallDetector/OctBall/BallDetector/analysis/LogBlenderCoord3DMove10.txt")

with open('D:/Sirius/FolderForBallDetector/OctBall/BallDetector/CameraParametrs.json') as f:
    templates = json.load(f)


rVecR = np.array(templates["extrinsics"]["rvecR"])
tVec = np.array ( templates["extrinsics"]["Tvec"]  ) 

distortion = np.array(templates["intrinsics"]["distortion"])
CameraMatrix = np.array(templates["intrinsics"]["K"])
CameraMatrix = CameraMatrix.reshape((3,3))

ProjectPointsFind, jacobFind = cv2.projectPoints(FindCord, rVecR, tVec,CameraMatrix, distortion)
ProjectPointsTrue, jacobTrue = cv2.projectPoints(TrueCord, rVecR, tVec,CameraMatrix, distortion)

plt.scatter(ProjectPointsFind[:,0,0] , ProjectPointsFind[:,0,1] )
plt.scatter(ProjectPointsTrue[:,0,0] , ProjectPointsTrue[:,0,1] )

plt.show()
cv2.waitKey(0)