# -*- coding: utf-8 -*-
"""
Created on Mon Feb 12 15:33:27 2018

@author: adity
"""

import cv2

import numpy as np


data_retrieve_path = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3"
#data_store_path = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/"

vector_2 = np.zeros((1,10800), dtype=np.uint8)#[[0:10800]]
vector_2 = np.array(vector_2)


for i in range (1,1001):
    data_retrieve_path = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3/"+'img_' + str(i) + '.jpg'
    
    image = cv2.imread(data_retrieve_path, 1) 
    
    img_face = np.array(image)
    
    vector_1 = img_face.flatten()
    vector_1 = np.array([vector_1])
    
    
    
    vector_2 = np.append(vector_2, vector_1, axis =0)
    
    
    
    
    
    print('i',i)

vector_2 = np.delete(vector_2,0,0)
vector_3 = vector_2.T

Mean = []#np.empty((1,1000), dtype=np.uint8)#[[0:10800]]
for i in range (0,10800):
    
    tmp_mean = np.mean(vector_3[i])
    
    Mean = np.append(Mean,tmp_mean)

Mean = np.array([Mean])    
mean_vis = np.reshape(Mean, (60,60,3)).astype('uint8')


tmp = np.reshape(vector_1, (60,60,3)).astype('uint8')

cv2.imshow('image', img_face)
cv2.imshow('image1', tmp)
cv2.imshow('image2', mean_vis)


k = cv2.waitKey(0)
cv2.destroyAllWindows()
   
    
    
    
    
    
    
    