#!/usr/bin/env python
# coding: utf-8

# In[1]:


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


# In[2]:


import numpy as np
import matplotlib.pyplot as plt
from ipywidgets import interact

FindCord = np.loadtxt(r"./BallDetectorLogMomentWithCameraMatrix.txt")
BlenderCord63 = np.loadtxt(r"./LogBlenderCoord63video.txt", delimiter=',')



# In[3]:


def paint(TrueCord = FindCord, FindCord = BlenderCord63):
    plt.plot(FindCord, label = "Find")
    plt.plot(TrueCord, label = "True")
    plt.grid()
    plt.legend()
    plt.show()


# In[4]:


w = interact(paint(BlenderCord63[:,0], FindCord[:,0]) )


# In[5]:





# In[ ]:




