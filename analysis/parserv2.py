#from turtle import dot
from calendar import c
from matplotlib import projections
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D

import numpy as np
def show3DPoints():
    ar = np.loadtxt("Undistort_by_Jenny.txt")

    fig = plt.figure()
    axes3d = Axes3D(fig)
    #plt.mplot3D(a, 'ro')
    A = 0.200641
    B =  0.0568383
    C = 0.978015
    D = -0.999491

    x = np.linspace(0,0.3,5)
    y = np.linspace(0,0.3,5)

    X,Y = np.meshgrid(x,y)
    Z = (- A*X - B*Y - D+0.01) / C 

    #Z = np.sqrt(X**2+Y**2)
    print (Z)
    axes3d.plot_surface(X,Y,Z)
    Z = (- A*X - B*Y - D-0.01) / C 
    axes3d.plot_surface(X,Y,Z)

    axes3d.set_xlabel('X')
    axes3d.set_ylabel('Y')
    axes3d.set_zlabel('Z')
    plt.ylim(0, 0.3)
    plt.xlim(0, 0.3)
    axes3d.scatter(ar[:,0], ar[:,1], ar[:,2],  cmap='viridis', linewidth=0.5, color = 'r')

    plt.show()
def show2DPoints():
    ar = np.loadtxt("Undistort.txt")
    fig = plt.figure(figsize=(8, 8), dpi=80)
    ax = fig.add_subplot()
    #fig, ax = plt.subplots()
    #ax.grid(True, which='both')
    ax.scatter(ar[:,0], ar[:,1],  cmap='viridis', linewidth=0.5, color = 'r')
    #ax.autoscale(False)  
    plt.ylim(0, 0.3)
    plt.xlim(0, 0.3)
    plt.show()
def ignore():
    ar = np.loadtxt("Undistort.txt")
    if (len(ar[0, :]) != len(ar[1, :])):
        print("ErrInputData")

    CenterX = sum(ar[0, :] / len(ar[0, :]))
    CenterY = sum(ar[1, :] / len(ar[1, :]))
    print (CenterX,CenterY)
    minX = min(ar[0,:])
    Radius = CenterX-minX
    print ("Radius  ",Radius)
    print ("Error: ", (CenterX + Radius) - max(ar[0,:]) )
    print ("Error: ", (CenterY + Radius) - max(ar[1,:]) )

def proektionOnPlane():
    A = 0.200641
    B =  0.0568383
    C = 0.978015
    D = -0.999491
    ar = np.loadtxt("Undistort.txt")

    normal = np.array([A,B,C])
    distantPointToPlane = normal.dot([ar[0, 0],ar[0, 1],ar[0,2]]) + D
    #print (distantPointToPlane)
    projectionPoint = [ar[0, 0],ar[0, 1],ar[0,2]] - distantPointToPlane* normal
    print ("Projection = ",projectionPoint)
    print("Point ", [ar[0, 0],ar[0, 1],ar[0,2]])

def OcenkaANSAK():
    A = 0.200641
    B =  0.0568383
    C = 0.978015
    D = -0.999491
    ar = np.loadtxt("Undistort.txt")
    normal = np.array([A,B,C])
    sumError = 0.0
    for i in range (len(ar)):
        sumError += normal.dot([ar[i, 0],ar[i, 1],ar[i,2]]) + D
    print("SumError", sumError)

def krug3DChek():
    ar = np.array(np.loadtxt("Undistort.txt"))
    center = sum (ar)/ar.shape[0]
    print("center", center)
    print(np.linalg.norm(center))
    R = sum(np.linalg.norm(ar - center, axis=1))/ar.shape[0]
    print("Radius", R)
    s = sum(np.power(np.linalg.norm(ar - center, axis=1) - R,2))/ar.shape[0]
    print("Error",s)


def myNorm(a=np.array([1,2,3]),b=np.array([2,2,4])):
    return np.linalg.norm(a - b, ord=2)

def Exp1(): # Radius = 0.15         4 light      z= const
    blenderCord1 = np.array([-0.5 , 0.3, 2.50]) 
    blenderCord2 = np.array([0.6, 0.2, 2.5])

    #findCord2 = np.array([-0.502347, 0.301249, 2.50983]) #Error 0.09784141727539401          PixError:   31.700619197227663
    #findCord1 = np.array([0.601789, -0.200045, 2.51009])
    #Without erode and dilate
    findCord1 = np.array([-0.502193, 0.301551, 2.50877]) # Error:  0.09806367275928345         PixError:   31.772629974007838
    findCord2 = np.array([0.602173, -0.200341, 2.51053]) 

    FindError = (abs(myNorm(blenderCord1,blenderCord2) - myNorm(findCord1,findCord2))) / abs(myNorm(blenderCord1,blenderCord2)) 
    print( "Error: ", FindError )
    print ("PixError:  ", (FindError) / (1/324))

def Exp2(): # Подбор по координа Z
    blenderCord1 =np.array([ 0.320215, 0.046521, 2.500000])
    findCord1 = np.array([0.320375, 0.0470963, 2.50537])
    
    result = [0.320595, 0.0472188, 2.50626]
    result = [0.320595, 0.0472188, 2.50626]
    FindError = (myNorm(blenderCord1,findCord1)) 
    print( "Error: max counter 0.619362 ", FindError )
def Gistogramm(data):
    fig, axs = plt.subplots(1, 2)
    n_bins = len(data)
    axs[0].hist(data['sepal length (cm)'], bins=n_bins)
    axs[0].set_title('sepal length')

krug3DChek()
#OcenkaANSAK()
#show2DPoints()
#proektionOnPlane()
#ignore()
show3DPoints()




########################################BlenderCord  = np.loadtxt('D:\Sirius\BallDetectorV2\BallDetectorV2\TestImagesLight2DMove\LogBlender.txt', dtype=float) # 100x3
########################################FindCord = a2 = np.loadtxt('D:\Sirius\BallDetectorV2\BallDetectorV2\TestImagesLight2DMove\FindCord.txt', dtype=float)
########################################sumErr = 0
########################################sumZErr=0
########################################sumXErr = 0
########################################sumYErr =0
########################################NormaBAndF = []
########################################for i in range(100):
########################################    sumErr += myNorm(BlenderCord[i,:], FindCord[i,:])
########################################    NormaBAndF.append( myNorm(BlenderCord[i,:], FindCord[i,:]))
########################################    sumZErr +=  abs(BlenderCord[i,2] - FindCord[i,2])
########################################    sumXErr +=  abs(BlenderCord[i,0] - FindCord[i,0])
########################################    sumYErr +=  abs(BlenderCord[i,1] - FindCord[i,1])
########################################
########################################NormaBAndF = np.asarray(NormaBAndF)
########################################logFile = open("D:/Sirius/BallDetectorV2/BallDetectorV2/TestImagesLight2DMove/LogNorma.txt", "w+" )
########################################np.savetxt(logFile, NormaBAndF, fmt ='%.8f' )
########################################print('Done')
########################################print ('SumZErr ', sumZErr/100)
########################################print ('SumXErr ', sumXErr/100)
########################################print ('SumYErr ', sumYErr/100)
########################################logFile.close()




#show3DPoints()
#OcenkaANSAK()


#Gistogramm(NormaBAndF)

    #sumErr += (abs(myNorm(BlenderCord[i,:],BlenderCord[i+1,:]) - myNorm(FindCord[i,:],FindCord[i+1,:]))) / abs(myNorm(BlenderCord[i,:],BlenderCord[i+1,:])) 
#print (myNorm(BlenderCord[0,:], FindCord[0,:]))

#################################################print ("Sum errors: ",sumErr)
#################################################AbsErr = sumErr/100
#################################################print ("median error ", AbsErr)
#################################################pixErr = pow(pow(0.15 / 324,2)*3,0.5)
#################################################print ("Pixsize  ", pixErr)
#################################################print ("Error in pixel: ", AbsErr/pixErr)
#blenderCord1 = np.array([0.6, 0.25, 2.50])
#blenderCord2 = np.array()
#findCord2 = np.array()
#findCord1 = np.array()


#sumzOtcl = 0
#for i in range (99):
#    sumzOtcl += abs(FindCord[i, 2] - FindCord[i+1, 2])
#print ("sumz: ", sumzOtcl/99)


#Проверка алгоритма смещения
##a = np.array([2, 2, 4])
##b = np.array([2, 3, 5])
##c =  np.array([5, 7, 2])
##d =  np.array([5, 6, 3])
##print ((myNorm(a, b) + myNorm(c,d) )/ 2)
##print ((abs(myNorm(a,c) - myNorm(b,d))) / abs(myNorm(a,c)) ) 
##radius = 0.15
##pixSize = (radius*2)/328
##


#Exp1()
#Exp2()


#print((abs(myNorm(blenderCord1,blenderCord2) - myNorm(findCord1,findCord2))) / abs(myNorm(blenderCord1,blenderCord2))  )

#[0.564243, -0.0551249, 2.50046] подгон Z
#result = [0.56401, -0.0551218, 2.50576] Точный Z
#[0.563132 m, -0.055514 m, -2.5]


#blenderCord1Z = np.array([0.563132, -0.055514, -2.5])
#findCord2Z = np.array([0.564243, -0.0551249, -2.50046])
#findCord1Z = np.array([0.56401, -0.0551218, -2.50576])
#print ('Podgon Norma' ,myNorm(blenderCord1Z,findCord2Z) )
#print ('Tochno Norma' ,myNorm(blenderCord1Z,findCord1Z ))

#При 0,006 смешенеие
#Sum errors:  0.11658664809543641
#median error  0.001165866480954364
#SumZErr  0.00021083700000000592
#SumXErr  0.0008508577819999982
#SumYErr  0.000615186159000001

#При смещении 0,005 = f
#SumZErr  0.0008513779999999826
#SumXErr  0.0008950788275000006
#SumYErr  0.0005344249309999996
#Sum errors:  0.14659894130230525
#median error  0.0014659894130230524

# smesh = 0.005=f and k small
#umZErr  0.0007234389999999902
#SumXErr  0.0008652505393000015
#SumYErr  0.0005240646769999994
#Sum errors:  0.13968136848605744
#median error  0.0013968136848605745
#Error in pixel:  2.979869194369226

#smeh = 0.005 and k = 0.08 count ~80
#SumZErr  0.0007209049999999984
#SumXErr  0.000880890804800004
#SumYErr  0.0005316151119999988
#Sum errors:  0.13931508373403645
#median error  0.0013931508373403645
#Error in pixel:  2.9720551196594442

#3dmove


#last
#SumZErr  0.0006742379999999937
#SumXErr  0.0007618974500000006
#SumYErr  0.0006172242191000001
#Sum errors:  0.1327295023673685
#median error  0.001327295023673685
#Pixsize   0.0008018753738744802
#Error in pixel:  1.655238540698034