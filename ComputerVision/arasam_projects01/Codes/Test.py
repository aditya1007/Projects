# -*- coding: utf-8 -*-
"""
Created on Sun Feb 11 16:01:29 2018

@author: adity
"""
import cv2
import pandas as pd
import numpy as np


a = [1,2,3,4,5,6,7,8,9]
b = [9,8,7,6,5,4,3,2,1]
x = [[]]

a=np.array([a])
b=np.array(b)

x=np.concatenate((a,x),axis=1)

d = np.array([[1,2,3,4,5,6,7,8,9]])
e = np.array([[30,8,7,6,5,4,3,2,1]])

f = np.concatenate((d,e), axis =0)

#m = np.concatenate((a,f), axis =0)

z =  f.flatten()
f = f.T

image = cv2.imread('image00019.jpg',1)

img = np.array(image)
img_1d = img.flatten()
#cv2.imshow('image', image)

#k = cv2.waitKey(0)
#cv2.destroyAllWindows()

pd.concat


tmp = np.array([ [[1,1,1],[1,1,1],[1,1,1]], [[5,5,5],[5,5,5],[5,5,5]], [[1,1,1],[1,1,1],[1,1,1]]])

#print(f[1:])

n= np.append(d,e, axis =0)
h = np.mean(f[0])
g = np.var(a)

p = np.array([1,2,3,4,5])

di = np.diag(p)
