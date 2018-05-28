# -*- coding: utf-8 -*-
"""
Created on Mon Feb 12 15:33:27 2018

@author: adity
"""

import cv2

import numpy as np
from numpy.linalg import inv
from numpy.linalg import det
from sklearn import preprocessing
from sklearn import decomposition
import scipy

import matplotlib.pyplot as plt
from sklearn import metrics


def perform_PCA(X,pca_components):
    pca = decomposition.PCA(n_components=pca_components)
    pca.fit(X)
    X_PCA = pca.transform(X)
    return X_PCA

#-----------------------------------DATA EXTRACTION-------------------------------------------------
    
    #-------------------------------VARIABLE INITIALIZATION-----------------------------------------
    
dis_size  = 60
Num_of_features_act  =  dis_size*dis_size*3


size = 5 #Image size after downsampling is = size*size*3  for RGB and = size*size for GRAYSCALE   
Num_of_features     =   size * size * 3 

vector_face_2       =   np.zeros((1,10800), dtype=np.uint8)#[[0:10800]]
vector_face_2       =   np.array(vector_face_2)

vector_nonface_2    =   np.zeros((1,10800), dtype=np.uint8)#[[0:10800]]
vector_nonface_2    =   np.array(vector_nonface_2)
    
vector_face_2_test       =   np.zeros((1,Num_of_features), dtype=np.uint8)#[[0:10800]]
vector_face_2_test       =   np.array(vector_face_2_test)

vector_nonface_2_test       =   np.zeros((1,Num_of_features), dtype=np.uint8)#[[0:10800]]
vector_nonface_2_test       =   np.array(vector_nonface_2_test)



    #-------------------------------TRAINING DATA---------------------------------------------------
for i in range (1,1001):
    
    
    data_retrieve_path_face     =   "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3/"+'img_' + str(i) + '.jpg'
    data_retrieve_path_nonface  =   "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/nonface3/"+'img_' + str(i) + '.jpg'
    
    image_w1        =   cv2.imread(data_retrieve_path_face, 1) 
    image_w0        =   cv2.imread(data_retrieve_path_nonface, 1) 
    
       
    
    img_face        =   np.array(image_w1)
    img_backgnd     =   np.array(image_w0)
    
    
    
    vector_face_1       = img_face.flatten()
    vector_face_1       = np.array([vector_face_1])
    
    vector_nonface_1    = img_backgnd.flatten()
    vector_nonface_1    = np.array([vector_nonface_1])
    
    
    
    vector_face_2       = np.append(vector_face_2, vector_face_1, axis =0)
    vector_nonface_2    = np.append(vector_nonface_2, vector_nonface_1, axis =0)
    
    
    
    
    print('i',i)


vector_face_2       = np.delete(vector_face_2,0,0)
vector_nonface_2    = np.delete(vector_nonface_2,0,0)


#--------------------------Add scaler---------------------------
vector_face_2_1       = perform_PCA(vector_face_2,Num_of_features)
vector_nonface_2_1    = perform_PCA(vector_nonface_2,Num_of_features)

#---------------------VECTOR with TRAINING IMAGE DATA-----------------------------------------------
vector_face_3       = vector_face_2_1.T               #Dim[Features x NoOfImages]
vector_nonface_3    = vector_nonface_2_1.T
#---------------------------------------------------------------------------------------------------






    #-----------------------------Testing Data Extraction-------------------------------------------
for i in range (1001,1101):

    
    Testdata_retrieve_path_face     =   "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3/"+'img_' + str(i) + '.jpg'
    image_path_test_face     =   Testdata_retrieve_path_face       
    
    Testdata_retrieve_path_nonface = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/nonface3/"+'img_' + str(i) + '.jpg'    
    image_path_test_nonface     =   Testdata_retrieve_path_nonface
        
    new_x_w1        =   cv2.imread(image_path_test_face, 1) 
    new_x_w0        =   cv2.imread(image_path_test_nonface, 1) 
    
    
    new_x_w1        =       cv2.resize(new_x_w1,(size,size),interpolation = cv2.INTER_CUBIC)
    new_x_w0        =       cv2.resize(new_x_w0,(size,size),interpolation = cv2.INTER_CUBIC)
    
    new_x_w1        =       np.array(new_x_w1)
    new_x_w0        =       np.array(new_x_w0)



    new_x_w0        =       new_x_w0.flatten()
    new_x_w1        =       new_x_w1.flatten()



    new_x_w1        =       np.array([new_x_w1])
    new_x_w0        =       np.array([new_x_w0])

    #new_x_w1        =       new_x_w1.T
    #new_x_w0        =       new_x_w0.T
    
    vector_face_2_test = np.append(vector_face_2_test, new_x_w1, axis =0)
    vector_nonface_2_test = np.append(vector_nonface_2_test, new_x_w0, axis =0)
    

vector_face_2_test       =      np.delete(vector_face_2_test,0,0)
vector_nonface_2_test    =      np.delete(vector_nonface_2_test,0,0)


#---------------------VECTOR with TESTING IMAGE DATA-----------------------------------------------
vector_face_3_test       =      (vector_face_2_test).T          #Dim[Features x NoOfImages]
vector_nonface_3_test    =     (vector_nonface_2_test).T
#---------------------------------------------------------------------------------------------------
    



#---------------------------DISPLAY of MEAN and COVARIANCE of TRAINING DATA----------------------

Mean_w1_dis     =   []#np.empty((1,1000), dtype=np.uint8)#[[0:10800]]
Cov_w1_dis      =   []

Mean_w0_dis     =   []#np.empty((1,1000), dtype=np.uint8)#[[0:10800]]
Cov_w0_dis      =   []

tmp_img_face        =   vector_face_2.T
tmp_img_nonface         =   vector_nonface_2.T

for i in range (0,Num_of_features_act):
   
    tmp_mean            =   np.mean(tmp_img_face[i]) 
    Mean_w1_dis          =   np.append(Mean_w1_dis,tmp_mean)
     
    tmp_mean                =   np.mean(tmp_img_nonface[i]) 
    Mean_w0_dis             =   np.append(Mean_w0_dis,tmp_mean)



for i in range (0,Num_of_features_act):
    
    tmp_cov         =       np.cov(tmp_img_face[i])    
    Cov_w1_dis      =       np.append(Cov_w1_dis,tmp_cov)
    
    tmp_cov         =       np.cov(tmp_img_nonface[i])  
    Cov_w0_dis      =       np.append(Cov_w0_dis,tmp_cov)



Cov_w1_dis          =       np.array(Cov_w1_dis) 
Cov_w1_dis          =       np.sqrt(Cov_w1_dis)
 

cov_vis_face        =       np.array([Cov_w1_dis]) #2-D vector
cov_vis_face        =       np.reshape(cov_vis_face, (dis_size,dis_size,3)).astype('uint8')
    
Cov_w0_dis          =       np.array(Cov_w0_dis) 
Cov_w0_dis          =       np.sqrt(Cov_w0_dis)
 

cov_vis_nonface     =       np.array([Cov_w0_dis]) #2-D vector
cov_vis_nonface     =       np.reshape(cov_vis_nonface, (dis_size,dis_size,3)).astype('uint8')
    
 
Mean_w1_dis         =       np.array([Mean_w1_dis])    
mean_vis_face       =       np.reshape(Mean_w1_dis, (dis_size,dis_size,3)).astype('uint8')

Mean_w0_dis         =       np.array([Mean_w0_dis])    
mean_vis_nonface    =       np.reshape(Mean_w0_dis, (dis_size,dis_size,3)).astype('uint8')



#cv2.imshow('image_mean_face', mean_vis_face)
#cv2.imwrite('image_mean_face.jpeg', mean_vis_face)
scipy.misc.imsave('mean_face.jpg', mean_vis_face)
#cv2.imshow('image_mean_nonface', mean_vis_nonface)
##cv2.imwrite('image_mean_nonface.jpeg',mean_vis_nonface)
scipy.misc.imsave('mean_nonface.jpg', mean_vis_nonface)
#cv2.imshow('image_cov_face', cov_vis_face)
##cv2.imwrite('image_cov_face.jpeg',cov_vis_face)

scipy.misc.imsave('image_cov_face.jpg', cov_vis_face)
#cv2.imshow('image_cov_nonface', cov_vis_nonface)
##cv2.imwrite('image_cov_nonface.jpeg',cov_vis_nonface)
scipy.misc.imsave('image_cov_nonface.jpg', cov_vis_nonface)


#k = cv2.waitKey(0)
#cv2.destroyAllWindows()





#---------------------------ESTIMATION of MEAN and COVARIANCE of TRAINING DATA----------------------

Mean_w1 = []#np.empty((1,1000), dtype=np.uint8)#[[0:10800]]
Cov_w1 = []

Mean_w0 = []#np.empty((1,1000), dtype=np.uint8)#[[0:10800]]
Cov_w0 = []
    



for i in range (0,Num_of_features):
    
    tmp_mean    =   np.mean(vector_face_3[i]) 
    Mean_w1     =   np.append(Mean_w1,tmp_mean)
    
    tmp_mean    =   np.mean(vector_nonface_3[i]) 
    Mean_w0     =   np.append(Mean_w0,tmp_mean)
    
Mean_w1 = np.array([Mean_w1])    


Mean_w0 = np.array([Mean_w0])    



for i in range (0,Num_of_features):
    
    tmp_cov = np.cov(vector_face_3[i])  
    Cov_w1 = np.append(Cov_w1,tmp_cov)
    
    tmp_cov = np.cov(vector_nonface_3[i])  
    Cov_w0 = np.append(Cov_w0,tmp_cov )
    
Cov_w1 = np.array(Cov_w1)    
cov_vis_face_diag = np.diag(Cov_w1)   #Diagonalising  


Cov_w0 = np.array(Cov_w0)    
cov_vis_nonface_diag = np.diag(Cov_w0)   #Diagonalising


  
    
mean_face = Mean_w1.T
mean_nonface = Mean_w0.T

cov_face = cov_vis_face_diag
cov_nonface = cov_vis_nonface_diag    
    
#-------------------------------Estimation of posterior---------------------------------------------

#P = []  
#P_w0 = []   
P_x1_w1_list = []           #
P_x1_w0_list = []

P_x0_w1_list = [] 
P_x0_w0_list = []

Pr_w0  =  0.9
Pr_w1  =  (1-Pr_w0)

#---Posterior---

P_w1_x1_list = [] 
P_w0_x1_list = []
P_w1_x0_list = [] 
P_w0_x0_list = []


P_w0_x_list = []
AAA = []

for i in range (0,100):
    
    
    
    image_face_new      =   vector_face_3_test[:,i]
    image_face_new      =   (np.array([image_face_new])).T
    
    
    image_nonface_new   =   vector_nonface_3_test[:,i]
    image_nonface_new   =   (np.array([image_nonface_new])).T


    #--------------------------FACE DATA (x1)------------------------------------------------------------
    
    #---------------------------MEAN & COV (w1)of FACE - Norm[M1,C1]------------------------------------
    #---------------------------TRUE POSITIVE------------------------
    f3_w1           =           (image_face_new - mean_face)  
    f2_w1           =           inv(cov_face)
    f1_w1           =           f3_w1.T
    
    tmp_index_w1    =           np.dot(f1_w1,f2_w1)
    index_w1        =           np.dot(tmp_index_w1,f3_w1)
    
    Pr_x1_w1         =           np.exp(-0.5*index_w1)
    
    P_x1_w1_list.append(Pr_x1_w1)

    #---------------------------MEAN & COV (w0) of NONFACE - Norm[M0,C0]------------------------------------
    #---------------------------TRUE Negative------------------------
    f3_w0           =           (image_face_new - mean_nonface)  
    f2_w0           =           inv(cov_nonface)
    f1_w0           =           f3_w0.T
    
    tmp_index_w0    =           np.dot(f1_w0,f2_w0)
    index_w0        =           np.dot(tmp_index_w0,f3_w0)
    
    Pr_x1_w0         =           np.exp(-0.5*index_w0)
    
    P_x1_w0_list.append(Pr_x1_w0)

    #--------------------------NONFACE DATA (x0)------------------------------------------------------------
    
    #---------------------------MEAN & COV (w1)of FACE - Norm[M1,C1]------------------------------------
    #---------------------------FALSE POSITIVE------------------------
    f3_w1           =           (image_nonface_new - mean_face)  
    f2_w1           =           inv(cov_face)
    f1_w1           =           f3_w1.T
    
    tmp_index_w1    =           np.dot(f1_w1,f2_w1)
    index_w1        =           np.dot(tmp_index_w1,f3_w1)
    
    Pr_x0_w1         =           np.exp(-0.5*index_w1)
    
    P_x0_w1_list.append(Pr_x0_w1)

    #---------------------------MEAN & COV (w0) of NONFACE - Norm[M0,C0]------------------------------------
    #---------------------------FALSE NEGATIVE------------------------
    f3_w0           =           (image_nonface_new - mean_nonface)  
    f2_w0           =           inv(cov_nonface)
    f1_w0           =           f3_w0.T
    
    tmp_index_w0    =           np.dot(f1_w0,f2_w0)
    index_w0        =           np.dot(tmp_index_w0,f3_w0)
    
    Pr_x0_w0         =           np.exp(-0.5*index_w0)
    
    P_x0_w0_list.append(Pr_x0_w0)



    den_face_Img        =       Pr_x1_w1 * Pr_w1    +   Pr_x1_w0 * Pr_w0    #---for true img
    den_nonface_Img     =       Pr_x0_w1 * Pr_w1    +   Pr_x0_w0 * Pr_w0    #---for false img

    #---------------------------TRUE POSITIVE------------------------
    Pr_w1_x1            =       Pr_x1_w1 * Pr_w1/den_face_Img
    
    P_w1_x1_list.append(Pr_w1_x1[0,0])
    
    
    #---------------------------TRUE Negative------------------------
    Pr_w0_x1            =       Pr_x1_w0 * Pr_w0/den_face_Img
    
    P_w0_x1_list.append(Pr_w0_x1[0,0])
    
    
    
    #---------------------------FALSE POSITIVE------------------------
    Pr_w1_x0            =       Pr_x0_w1 * Pr_w1/den_nonface_Img
    
    P_w1_x0_list.append(Pr_w1_x0[0,0])
    
    #---------------------------FALSE NEGATIVE------------------------
    Pr_w0_x0            =       Pr_x0_w0 * Pr_w0/den_nonface_Img
    
    P_w0_x0_list.append(Pr_w0_x0[0,0])
    
#----True = x1
#----False = x0
#----Positive = Mean & Covar of Face w1
#----Negative = Mean & Covar of NonFace w0

P_w1_x1_array = np.array(P_w1_x1_list)
P_w0_x1_array = np.array(P_w0_x1_list)
P_w1_x0_array = np.array(P_w1_x0_list)
P_w0_x0_array = np.array(P_w0_x0_list)



True_Neg    = P_w1_x1_array
False_Neg    = P_w0_x1_array

Posterior = np.append(True_Neg,False_Neg)
labels = np.append(np.ones(100), np.zeros(100))

fpr, tpr, _ = metrics.roc_curve(labels,Posterior, pos_label=0)
plt.plot(fpr, tpr, color='darkorange')
plt.xlim([0,1])
plt.ylim([0,1])
plt.show()
  
True_Neg    = P_w0_x0_array
False_Neg    = P_w1_x0_array

Posterior = np.append(True_Neg,False_Neg)
labels = np.append(np.ones(100), np.zeros(100))

fpr, tpr, _ = metrics.roc_curve(labels,Posterior, pos_label=0)
plt.plot(fpr, tpr, color='darkorange')
plt.xlim([0,1])
plt.ylim([0,1])
plt.show()

















PosFace = 0
NegFace = 0
PosNonFace = 0
NegNonFace = 0

for i in range (0,100):
    if P_w1_x1_array[i] < 0.5:
        PosFace = PosFace + 1
    PosFace = 0

    if P_w0_x1_array[i] > 0.5:
        NegFace = NegFace + 1
    
    if P_w1_x0_array[i] > 0.5:
        PosNonFace = PosNonFace + 1
        
    if P_w0_x0_array[i] < 0.5:
        NegNonFace = NegNonFace + 1
        
FalsePositiveRate = NegFace/100
FalseNegativeRate = PosNonFace/100

MisclassificationRate =   FalsePositiveRate+    FalseNegativeRate
print(FalsePositiveRate)
print(FalseNegativeRate)
print(MisclassificationRate)

       
        