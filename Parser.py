from enum import Flag
from lib2to3.pytree import convert
import matplotlib.pyplot as plt

def original():
    PointsX = []
    PointsY = []
    with open('original.txt','r') as f:
        for line in f:
            j = False
            for word in line.split(","):
                if j == False:
                    PointsX.append(word)
                else:
                    PointsY.append(word)
                j = True
    PointsY = [x.strip() for x in PointsY] 
    PointsY = [x.strip(';') for x in PointsY] 
    PointsY = [x.strip(']') for x in PointsY] 

    PointsX = [x.strip() for x in PointsX] 
    PointsX = [x.strip(';') for x in PointsX] 
    PointsX = [x.strip('[') for x in PointsX] 

    for i in range (len(PointsY)):
        print (PointsX[i])
    PointsX = [float(x) for x in PointsX]
    PointsY = [float(x) for x in PointsY]

    plt.plot(PointsX, PointsY, 'bo')
    
PointsX = []
PointsY = []
PointsZ = []
flag = 1
with open('Undistort.txt','r') as f:
    for line in f:
        aaa = line.split(",")
        for word in aaa:
            if flag == 1:
                PointsX.append(word)
                j = 1
                flag = 2
                break
            elif flag == 2:
                PointsY.append(word)
                print (word)
                flag = 3
            elif flag == 3:
                PointsZ.append(word)
                flag = 1
            
PointsY = [x.strip(' ') for x in PointsY] 
PointsY = [x.strip(';') for x in PointsY] 
PointsY = [x.strip(']') for x in PointsY] 

PointsX = [x.strip() for x in PointsX] 
PointsX = [x.strip(';') for x in PointsX] 
PointsX = [x.strip('[') for x in PointsX] 
PointsX = [x.strip(']') for x in PointsX] 

PointsZ = [x.strip() for x in PointsZ] 
PointsZ = [x.strip(';') for x in PointsZ] 
PointsZ = [x.strip('[') for x in PointsZ] 
PointsZ = [x.strip(']') for x in PointsZ] 

for i in range (len(PointsY)):
     print (PointsZ[i])
PointsX = [float(x) for x in PointsX]
PointsY = [float(x) for x in PointsY]
PointsZ = [float(x) for x in PointsZ]
plt.figure(figsize=(9, 9))
plt.plot(PointsX, PointsY, 'ro')

#original()

#plt.axis([0, 1600, 0, 1600])
plt.show()