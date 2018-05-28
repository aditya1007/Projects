# -*- coding: utf-8 -*-
"""
Created on Mon Feb 12 15:33:27 2018

@author: adity
"""

import cv2

import numpy as np


#data_retrieve_path = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3"
#data_store_path = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/"

vector_face_2 = np.zeros((1,10800), dtype=np.uint8)#[[0:10800]]
vector_face_2 = np.array(vector_face_2)

vector_nonface_2 = np.zeros((1,10800), dtype=np.uint8)#[[0:10800]]
vector_nonface_2 = np.array(vector_nonface_2)

for i in range (1,1001):
    data_retrieve_path_face = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3/"+'img_' + str(i) + '.jpg'
    data_retrieve_path_nonface = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/nonface3/"+'img_' + str(i) + '.jpg'
    
    image_w1 = cv2.imread(data_retrieve_path_face, 1) 
    image_w0 = cv2.imread(data_retrieve_path_nonface, 1) 
    
    
    
    img_face = np.array(image_w1)
    img_backgnd = np.array(image_w0)
    
    vector_face_1 = img_face.flatten()
    vector_face_1 = np.array([vector_face_1])
    
    vector_nonface_1 = img_backgnd.flatten()
    vector_nonface_1 = np.array([vector_nonface_1])
    
    
    
    vector_face_2 = np.append(vector_face_2, vector_face_1, axis =0)
    vector_nonface_2 = np.append(vector_nonface_2, vector_nonface_1, axis =0)
    
    
    
    
    print('i',i)

vector_face_2 = np.delete(vector_face_2,0,0)
vector_nonface_2 = np.delete(vector_nonface_2,0,0)



vector_face_3 = vector_face_2.T
vector_nonface_3 = vector_nonface_2.T

Mean_w1 = []#np.empty((1,1000), dtype=np.uint8)#[[0:10800]]
Cov_w1 = []

Mean_w0 = []#np.empty((1,1000), dtype=np.uint8)#[[0:10800]]
Cov_w0 = []


for i in range (0,10800):
    
    tmp_mean = np.mean(vector_face_3[i]) 
    Mean_w1 = np.append(Mean_w1,tmp_mean)
    
    tmp_mean = np.mean(vector_nonface_3[i]) 
    Mean_w0 = np.append(Mean_w0,tmp_mean)
    
    
    
    
    

Mean_w1 = np.array([Mean_w1])    
mean_vis_face = np.reshape(Mean_w1, (60,60,3)).astype('uint8')

Mean_w0 = np.array([Mean_w0])    
mean_vis_nonface = np.reshape(Mean_w0, (60,60,3)).astype('uint8')






for i in range (0,10800):
    
    tmp_cov = np.cov(vector_face_3[i])  
    Cov_w1 = np.append(Cov_w1,tmp_cov)
    
    tmp_cov = np.cov(vector_nonface_3[i])  
    Cov_w0 = np.append(Cov_w0,tmp_cov)



Cov_w1 = np.array(Cov_w1)    
cov_vis_face_diag = np.diag(Cov_w1)   #Diagonalising

cov_vis_face = np.sqrt(cov_vis_face_diag)

cov_vis_face = np.diag(cov_vis_face)  #Returns the diagonal element

cov_vis_face = np.array([cov_vis_face]) #2-D vector
cov_vis_face = np.reshape(cov_vis_face, (60,60,3)).astype('uint8')


Cov_w0 = np.array(Cov_w0)    
cov_vis_nonface_diag = np.diag(Cov_w0)   #Diagonalising

cov_vis_nonface = np.sqrt(cov_vis_nonface_diag)

cov_vis_nonface = np.diag(cov_vis_nonface)  #Returns the diagonal element

cov_vis_nonface = np.array([cov_vis_nonface]) #2-D vector
cov_vis_nonface = np.reshape(cov_vis_nonface, (60,60,3)).astype('uint8')


cv2.imshow('image2', mean_vis_face)
cv2.imshow('image3', cov_vis_face)

cv2.imshow('image4', mean_vis_nonface)
cv2.imshow('image5', cov_vis_nonface)


k = cv2.waitKey(0)
cv2.destroyAllWindows()
   
    
    
    
    
    
    
    