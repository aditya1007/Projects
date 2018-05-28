# -*- coding: utf-8 -*-
"""
Created on Sun Feb 25 23:35:55 2018

@author: adity
"""

import cv2
import inspect
import numpy as np
from numpy.linalg import inv
from numpy.linalg import det
from sklearn.preprocessing import StandardScaler

from scipy.stats import multivariate_normal
from scipy import optimize
import scipy
from math import * 
import math
import matplotlib.pyplot as plt
from sklearn import metrics
from sklearn import preprocessing
from sklearn import decomposition
local_vars = {}
local_vars_E_step_t_dist = {}
local_vars_M_step_t_dist = {}
local_vars_Init = {}
local_vars_Init_Likelihood  =  {}

#--------------------------------------------Function Declaration-----------------------------------
def perform_PCA(X,pca_components):
    pca = decomposition.PCA(n_components=pca_components)
    pca.fit(X)
    X_PCA = pca.transform(X)
    return X_PCA


def  Initialize_t_dist(NumOfFeatures, NumOfImages, K):
    global local_vars_Init
        
    #NumOfModels = 3 # K
    #NumOfFeatures = 75
    
    mean        =       np.random.randint(1,255,(NumOfFeatures,1))
    Phi         =       np.random.randint(1,255,(NumOfFeatures,K))
    covar       =       np.random.randint(1000,5000,(NumOfFeatures,1))
    covar       =       np.diagflat(covar)
    
    
   
    
    
    
    local_vars_Init    =       inspect.currentframe().f_locals     
    return mean, covar, Phi

        
def E_step_t_dist(img_x, Mean, Covar, Phi, NumOfFeatures , NumOfImages, K):
    global local_vars_E_step_t_dist
    x       =       img_x
    D       =       NumOfFeatures
    I       =       NumOfImages
    
    #d       =       np.zeros((1,I))
    E_hi       =       np.zeros((K,I))
    E_hi_hiT   =       np.zeros((I,K,K))
    
    IM      =       np.identity(K)
    
    mean1   =   np.array(Mean)
    
    mean1 = np.mean(x,axis = 1)
    mean1 = np.array([mean1])
    mean1 = mean1.T    
    #for i in range(D):
        #cov1_nf_temp[i]=np.cov(p_nf[i])
        #mean1[i,0]=np.mean(x[i,:])
        
    mean1 = mean1
    #cov1_nf=np.diagflat(cov1_nf_temp)
    
    for i in range (0,I):
        tmp_x   =   np.array([x[:,i]])
        tmp_x   =   tmp_x.T
        #tmp_x   =   (np.array([tmp_x])).T
        
        
        #mean1   =   np.array(Mean)
        covar1  =   Covar[:,:]           #---------------------------------*1000
        #covar1 = np.array([covar1])


        fact_3_m1   =   Phi
        fact_2_m1   =   inv(covar1)
        fact_1_m1   =   Phi.T
        
        tmp_fact_x1    =   np.dot(fact_1_m1,fact_2_m1)
        tmp_fact_x2    =   np.dot(tmp_fact_x1,fact_3_m1)   
        tmp_fact_x3    =   tmp_fact_x2 + IM
        tmp_fact    =   inv(tmp_fact_x3)
        
        fact_3_m1   =   (tmp_x - mean1)
        
        tmp_fact_1  =   np.dot(fact_1_m1,fact_2_m1)
        tmp_fact_1_1  =   np.dot(tmp_fact_1,fact_3_m1) 
        
        tmp_e           =       np.dot(tmp_fact,tmp_fact_1_1)
        
        E_hi[:,i]   =   (tmp_e).T # np.dot(tmp_fact,tmp_fact_1_1)
        E_hi_hiT[i,:,:]  =   tmp_fact + np.dot(E_hi[:,i] , E_hi[:,i] .T)
        
        
        
        #-----------------------M-Step-------------------------------------------------------------
        Phi11 = np.zeros((D,K))
        Phi12 = np.zeros((D,K))
        Phi13 = np.zeros((K,K))
        
        for i in range (0,I):
            tmp_x   =   np.array([x[:,i]])
            tmp_x   =   tmp_x.T
            
            fact_3_m1   =   (tmp_x - mean1)
            fact_1      =   np.array([E_hi[:,i]])
            #fact_1 = fact_1.T
            
            Phi11 = (np.dot(fact_3_m1,fact_1))
            Phi12 = Phi12 + Phi11
            
            #Phi13  =  E_hi_hiT[i,:,:]
            
            Phi13  =  Phi13  +  E_hi_hiT[i,:,:]
        
        
        Phi13 = inv(Phi13)
        
        Phi = np.dot(Phi12, Phi13)
        
        cov11 = np.zeros((D,D))
        cov12 = np.zeros((D,D))
        cov  =  np.zeros((D,D))
        for i in range (0,I):
            tmp_x   =   np.array([x[:,i]])
            
            
            
            fact_3_m1   =   (tmp_x.T - mean1)
            fact_3_m2   =    fact_3_m1.T
            tmp_xT   =   tmp_x.T
            
            cov11 = np.dot(fact_3_m1,fact_3_m2)
            
            cov12 = np.dot(Phi,E_hi[:,i,None])
            
            cov13 = np.dot(cov12,fact_3_m2)
        
            cov     =   cov + (cov11 - cov13)
            #cov11 = cov11 + np.dot(tmp_xT,tmp_x)
        
        cov = np.diag(cov)
        cov = cov[None]/I
        #cov1 = np.array([cov1])
        
        cov = np.diagflat(cov)  
            
        
    local_vars_E_step_t_dist        =       inspect.currentframe().f_locals       
    return Phi, cov, mean1



#------------------------------Likelihood function---------------------------------------------------
def Likelihd (img_x, mean, covar, NumOfFeatures, NoOfTestImages):
    global local_vars_Init_Likelihood
    x_t = img_x.T           #vector_face_3_test.T
    D = NumOfFeatures
    tmp_PrX = []
    
    for i in range (0,100):
        
    
        
            
        
        
        
        fact_t = np.dot((x_t[i,:]-mean),inv(covar))
        
        fact_t2 = np.dot(fact_t,(x_t[i,:]-mean).T)
        
        fact_t3 = np.exp((-0.5*fact_t2))
        
        
        tmp_PrX.append(fact_t3)
    
    Prx = np.array(tmp_PrX)
    local_vars_Init_Likelihood = inspect.currentframe().f_locals     
    return Prx

def scaling(img):
    scalar_face_test =StandardScaler()
    scalar_face_test.fit(img)
    img = scalar_face_test.transform(img)
    return img






#-----------------------------------DATA EXTRACTION-------------------------------------------------
    
    #-------------------------------VARIABLE INITIALIZATION-----------------------------------------
    
dis_size  = 60
down_size = 6
Num_of_features_act  =  dis_size*dis_size*3         #----------for display purpose
NumOfModels, NumOfFeatures, NoOfImages_train = 3, 75, 1000
K  = 2
RGB = 0

if RGB == 1:
    channels = 3
else:
    channels = 1

size = 5 #Image size after downsampling is = size*size*3  for RGB and = size*size for GRAYSCALE   
Num_of_features     =   size * size * channels 
NumOfFeatures       =   Num_of_features



Pixels = down_size*down_size*channels
    #--------------------------------VECTOR INITIALIZATION------------------------------------------
vector_face_2       =   np.zeros((1,Pixels), dtype=np.uint8)#[[0:10800]]
vector_face_2       =   np.array(vector_face_2)

vector_nonface_2    =   np.zeros((1,Pixels), dtype=np.uint8)#[[0:10800]]
vector_nonface_2    =   np.array(vector_nonface_2)
    
vector_face_2_test       =   np.zeros((1,Pixels), dtype=np.uint8)#[[0:10800]]
vector_face_2_test       =   np.array(vector_face_2_test)

vector_nonface_2_test       =   np.zeros((1,Pixels), dtype=np.uint8)#[[0:10800]]
vector_nonface_2_test       =   np.array(vector_nonface_2_test)



    #-------------------------------TRAINING DATA---------------------------------------------------
for i in range (1,1001):
    
    
    data_retrieve_path_face     =   "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3/"+'img_' + str(i) + '.jpg'
    data_retrieve_path_nonface  =   "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/nonface3/"+'img_' + str(i) + '.jpg'
    
    image_w1        =   cv2.imread(data_retrieve_path_face, RGB) 
    image_w0        =   cv2.imread(data_retrieve_path_nonface, RGB) 
    
    image_w1 = cv2.resize(image_w1,(down_size,down_size),interpolation = cv2.INTER_CUBIC)
    image_w0 = cv2.resize(image_w0,(down_size,down_size),interpolation = cv2.INTER_CUBIC)
       
    
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

'''
scalar_face =StandardScaler()
scalar_face.fit(vector_face_2_1)
vector_face_2_1 = scalar_face.transform(vector_face_2_1)
'''
#-----------------------------------------Add scalar for  nonface-----------------------------------
vector_face_2_1 = scaling(vector_face_2_1)

vector_nonface_2_1 = scaling(vector_nonface_2_1)



#---------------------VECTOR with TRAINING IMAGE DATA-----------------------------------------------
vector_face_3       = vector_face_2_1.T               #Dim[Features x NoOfImages]
vector_nonface_3    = vector_nonface_2_1.T
#---------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------





    #-----------------------------Testing Data Extraction-------------------------------------------
for i in range (1001,1101):

    
    Testdata_retrieve_path_face     =   "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3/"+'img_' + str(i) + '.jpg'
    image_path_test_face     =   Testdata_retrieve_path_face       
    
    Testdata_retrieve_path_nonface = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/nonface3/"+'img_' + str(i) + '.jpg'    
    image_path_test_nonface     =   Testdata_retrieve_path_nonface
        
    new_x_w1        =   cv2.imread(image_path_test_face, RGB) 
    new_x_w0        =   cv2.imread(image_path_test_nonface, RGB) 
    
    
    new_x_w1        =       cv2.resize(new_x_w1,(down_size,down_size),interpolation = cv2.INTER_CUBIC)
    new_x_w0        =       cv2.resize(new_x_w0,(down_size,down_size),interpolation = cv2.INTER_CUBIC)
    
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




vector_face_2_1_test       = perform_PCA(vector_face_2_test,Num_of_features)
vector_nonface_2_1_test    = perform_PCA(vector_nonface_2_test,Num_of_features)



    

'''
scalar_face_test =StandardScaler()
scalar_face_test.fit(vector_face_2_1_test)
vector_face_2_1_test = scalar_face_test.transform(vector_face_2_1_test)
'''
vector_face_2_1_test = scaling(vector_face_2_1_test)

vector_nonface_2_1_test = scaling(vector_nonface_2_1_test)

#---------------------VECTOR with TESTING IMAGE DATA-----------------------------------------------
vector_face_3_test       =      (vector_face_2_1_test).T          #Dim[Features x NoOfImages]
vector_nonface_3_test    =      (vector_nonface_2_1_test).T
#---------------------------------------------------------------------------------------------------



#x = vector_face_3

def ModelTrain(img_x):
    
    Mean, Covar, Phi = Initialize_t_dist(NumOfFeatures, NoOfImages_train, K)
    
    for i in range (0,2):
        i_Phi, i_cov, i_mean = E_step_t_dist(x, Mean, Covar, Phi, NumOfFeatures , NoOfImages_train, K)
        
        Covar = i_cov
        Phi   = i_Phi
    
    
    tmp_fact1 = np.dot(Phi,Phi.T)
    Combined_Covar = tmp_fact1 + Covar
    
    return Combined_Covar, i_mean


x = vector_face_3
covar_f_face,      mean_f_face,       =  ModelTrain(x)

x = vector_nonface_3
covar_f_nonface,      mean_f_nonface,       =  ModelTrain(x)



#--------------------------------Likelihood for TEST FACE ---------------------------------------
x_t = vector_face_3_test
Prx1_w1  =  Likelihd (x_t, mean_f_face, covar_f_face, NumOfFeatures, 100)

x_t = vector_face_3_test
Prx1_w0  =  Likelihd (x_t, mean_f_nonface, covar_f_nonface, NumOfFeatures, 100)

Prx1_w1 = Prx1_w1.T
Prx1_w0 = Prx1_w0.T



#--------------------------------Likelihood for TEST NONFACE ---------------------------------------
x_t = vector_nonface_3_test
Prx0_w1  =  Likelihd (x_t, mean_f_face, covar_f_face, NumOfFeatures, 100)

x_t = vector_nonface_3_test
Prx0_w0  =  Likelihd (x_t, mean_f_nonface, covar_f_nonface, NumOfFeatures, 100)

Prx0_w1 = Prx0_w1.T
Prx0_w0 = Prx0_w0.T


Prior_w1 = 0.5
Prior_w0 = 1 - Prior_w1



#----------------------------Code for posterior P_w_x------------------------------
Post_w0_x1  = np.zeros((1,100))
Post_w1_x1  = np.zeros((1,100))

Post_w0_x0  = np.zeros((1,100))
Post_w1_x0  = np.zeros((1,100))


True_Neg    =   []#np.zeros((100,1))
False_Neg    =  []# np.zeros((100,1))
True_Neg_1    =   []#np.zeros((100,1))
False_Neg_1    =  []# np.zeros((100,1))

for i in range (0,100):
    
    num_w1 = Prx1_w1[0,0,i] * Prior_w1
    num_w0 = Prx1_w0[0,0,i] * Prior_w0
    
    den = Prx1_w1[0,0,i] * Prior_w1 + Prx1_w0[0,0,i] * Prior_w0
    
    Post_w0_x1[0,i] = num_w0/den        #------------face - nonface model
    Post_w1_x1[0,i] = num_w1/den        #------------face - face model
    
    
    num_w1 = Prx0_w1[0,0,i] * Prior_w1
    num_w0 = Prx0_w0[0,0,i] * Prior_w0
    
    den = Prx0_w1[0,0,i] * Prior_w1 + Prx0_w0[0,0,i] * Prior_w0
    
    Post_w0_x0[0,i] = num_w0/den        #------------nonface - nonface model
    Post_w1_x0[0,i] = num_w1/den        #------------nonface - face model
    
    
    True_Neg.append(Post_w1_x1[0,i])
    False_Neg.append(Post_w0_x1[0,i])
    
    True_Neg_1.append(Post_w0_x0[0,i])
    False_Neg_1.append(Post_w1_x0[0,i])
    

True_Neg    = np.array(True_Neg)
False_Neg    = np.array(False_Neg)

Posterior = np.append(True_Neg,False_Neg)
labels = np.append(np.ones(100), np.zeros(100))

fpr, tpr, _ = metrics.roc_curve(labels,Posterior, pos_label=0)
plt.plot(fpr, tpr, color='darkorange')
plt.xlim([0,1])
plt.ylim([0,1])
plt.show()

   #---------------------------------------------------------------------------------------------
True_Neg_1    = np.array(True_Neg_1)
False_Neg_1    = np.array(False_Neg_1)

Posterior1 = np.append(True_Neg_1,False_Neg_1)
labels1 = np.append(np.ones(100), np.zeros(100))

fpr1, tpr1, _ = metrics.roc_curve(labels1,Posterior1, pos_label=0)
plt.plot(fpr1, tpr1, color='darkorange')
plt.xlim([0,1])
plt.ylim([0,1])
plt.show()   


PosFace = 0
NegFace = 0
PosNonFace = 0
NegNonFace = 0
Thresh = 0.1#10**(-15)

for i in range (0,100):
    if True_Neg[i] < Thresh:
        PosFace = PosFace + 1
    PosFace = 0

    if False_Neg[i] > Thresh:
        NegFace = NegFace + 1
    
    if False_Neg_1[i] > Thresh:
        PosNonFace = PosNonFace + 1
        
    if True_Neg_1[i] < Thresh:
        NegNonFace = NegNonFace + 1
        
FalsePositiveRate = NegFace/100
FalseNegativeRate = PosNonFace/100

MisclassificationRate =   FalsePositiveRate+    FalseNegativeRate
print(FalsePositiveRate)
print(FalseNegativeRate)
print(MisclassificationRate)