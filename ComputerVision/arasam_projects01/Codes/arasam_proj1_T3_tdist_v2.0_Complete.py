# -*- coding: utf-8 -*-
"""
Created on Fri Feb 23 13:32:26 2018

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
from scipy import special
import scipy
from math import * 

import matplotlib.pyplot as plt
from sklearn import metrics


from sklearn import preprocessing
from sklearn import decomposition
local_vars = {}
local_vars_E_step_t_dist = {}
local_vars_M_step_t_dist_new = {}
local_vars_Init = {}
local_vars_Init_Likelihood  =  {}

#--------------------------------------------Function Declaration-----------------------------------
def perform_PCA(X,pca_components):
    pca = decomposition.PCA(n_components=pca_components)
    pca.fit(X)
    X_PCA = pca.transform(X)
    return X_PCA


def  Initialize_t_dist(NumOfFeatures, NumOfImages):
    global local_vars_Init
        
    #NumOfModels = 3 # K
    #NumOfFeatures = 75
    
    mean        =       np.random.randint(1,255,(1,NumOfFeatures))
    covar       =       np.random.randint(1000,5000,(NumOfFeatures,1))
    covar       =       np.diagflat(covar)
    
    
    v           =       2
    #lmbda = np.random.rand(1,NumOfModels)
    
    
    
    local_vars_Init    =       inspect.currentframe().f_locals     
    return mean, covar, v

        
def E_step_t_dist(img_x, Mean, Covar, V, NumOfFeatures , NumOfImages):
    global local_vars_E_step_t_dist
    x       =       img_x
    D       =       NumOfFeatures
    I       =       NumOfImages
    
    d       =       np.zeros((1,I))
    E       =       np.zeros((1,I))
    E_log   =       np.zeros((1,I))
    
    for i in range (0,I):
        tmp_x   =   x[:,i]
        tmp_x   =   (np.array([tmp_x])).T
        
        
        mean1   =   (np.array([Mean[0,:]])).T
        covar1  =   Covar[:,:]           #---------------------------------*1000
        #covar1 = np.array([covar1])


        fact_3_m1   =   (tmp_x - mean1)
        fact_2_m1   =   inv(covar1)
        fact_1_m1   =   fact_3_m1.T
        
        tmp_fact    =   np.dot(fact_1_m1,fact_2_m1)
        d[0,i]      =   np.dot(tmp_fact,fact_3_m1)      
        
        E[0,i]      =   (V+D)/(V+d[0,i])
        E_log[0,i]  =   scipy.special.digamma((0.5*V+0.5*D)) - np.log(0.5*V+0.5*d[0,i])
        
        
        
    local_vars_E_step_t_dist        =       inspect.currentframe().f_locals       
    return E, E_log, d

def M_step_t_dist(img_x, V, E , E_log, NumOfFeatures, NumOfImages):
    
    global local_vars_M_step_t_dist_new
    
    v       =   V
    I       =   NumOfImages
    x       =   img_x.T
    mean    =   np.dot(E,x)/np.sum(E)
    
    covar   =  np.zeros((NumOfFeatures,NumOfFeatures))
    tcost   =  0
    for i in range (0,I):
        tmp_x   =   x[i,:]
        tmp_x   =   (np.array([tmp_x])).T
        
        
        #------------------COVARIANCE------------------------
        fact_3_m1   =   (tmp_x - mean)
            
        fact_1_m1   =   fact_3_m1.T
        tmp_fact    =   np.dot(fact_3_m1, fact_1_m1) 
        
        tmp_covar   =   E[0,i]*tmp_fact
        
        den         =   1+E[0,i]            #--------+1
                
        #-----------------Summing up ------------------------
        covar       =   (covar + tmp_covar)/ den
        
        def tcost(v):
            tcost = 0
            
            for k in range (0,2000):
                
                
                f1          =   0.5*v*np.log(v/2)
                f2          =   np.log(special.gamma(v*0.5))
                f3          =   (0.5*v-1)*E_log[0,i]
                f4          =   0.5 * E[0,i]
                
                tmp_tcost   =    f1 + f2 -f3 +f4
                
                tcost = tcost + tmp_tcost
                tcost = -tcost
            return tcost
    
    
        
        
    #tcost = tcost    
    v = optimize.fmin(tcost,2) 
   
    
    local_vars_M_step_t_dist_new = inspect.currentframe().f_locals 
    return mean, covar, v

#-----------------------------------DATA EXTRACTION-------------------------------------------------
    
    #-------------------------------VARIABLE INITIALIZATION-----------------------------------------
    
dis_size  = 60
down_size = 6
Num_of_features_act  =  dis_size*dis_size*3         #----------for display purpose
NumOfModels, NumOfFeatures, NoOfImages_train = 3, 75, 1000

RGB = 1

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

scalar_face =StandardScaler()
scalar_face.fit(vector_face_2_1)
vector_face_2_1 = scalar_face.transform(vector_face_2_1)

#-----------------------------------------Add scalar for  nonface-----------------------------------




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


scalar_face_test =StandardScaler()
scalar_face_test.fit(vector_face_2_1_test)
vector_face_2_1_test = scalar_face_test.transform(vector_face_2_1_test)





#---------------------VECTOR with TESTING IMAGE DATA-----------------------------------------------
vector_face_3_test       =      (vector_face_2_1_test).T          #Dim[Features x NoOfImages]
vector_nonface_3_test    =      (vector_nonface_2_1_test).T
#---------------------------------------------------------------------------------------------------







def ModelTrain(img_x):
    MEAN, COVAR, V   =       Initialize_t_dist(NumOfFeatures, NoOfImages_train)
    mean_t0, covar_t0, v_t0, = MEAN, COVAR, V
    
    Convergence = False    
    
    converg_cntr  = 0   
    tolerance_v = 0.1
    
    while (Convergence == False):
            #if first_run == 1: 
                
                
                
            #===================E_STEP=================================================    
            E_t1, E_log_t1, d_t1 = E_step_t_dist(x, mean_t0, covar_t0, v_t0, NumOfFeatures , NoOfImages_train)
            
            
            #===================M_STEP=================================================
            
            mean_t1, covar_t1, v_t1 = M_step_t_dist(x, v_t0, E_t1 , E_log_t1, NumOfFeatures, NoOfImages_train)
            
            converg_cntr    =   converg_cntr     +   1
            print('====================Counter:',converg_cntr)
            
           
            
            #mean_t0, covar_t0, lmbda_t0 = mean_t1, covar_t1, lmbda_t1 
            
            
            
            if (abs(v_t1 - v_t0) < tolerance_v) or (converg_cntr > 1):
                Convergence = True
                v_f,    mean_f,     covar_f         =      v_t1,    mean_t1,   covar_t1
            else:
                v_t0,    mean_t0,     covar_t0      =      v_t1,    mean_t1,   covar_t1 
    
    return v_f, mean_f, covar_f  







def Likelihd (img_x, v, mean, covar, NumOfFeatures, NoOfTestImages):
    global local_vars_Init_Likelihood
    x_t = img_x.T           #vector_face_3_test.T
    D = NumOfFeatures
    tmp_PrX = []
    
    for i in range (0,100):
        
    
        num_1 = gamma((v+D)*0.5)
            
        
        
        
        fact_t = np.dot((x_t[i,:]-mean),inv(covar))
        
        num_2 = np.dot(fact_t,(x_t[i,:]-mean).T)
        
        fact_t2 = (1+(num_2/v))
        
        index = -(v+D)/2
        
        tmp_Pr_xt = num_1 * fact_t2**index
        
        #Prx
        
        den = (gamma(v/2))*(v * pi)**(D/2)
        
        Prob = tmp_Pr_xt/den
        
        deter = det(covar)
    
        deter = np.sqrt(deter)
        tmp_PrX.append(Prob/deter)
    
    Prx = np.array(tmp_PrX)
    local_vars_Init_Likelihood = inspect.currentframe().f_locals     
    return Prx
#---------------------TRAINING MODEL with FACE DATA-------------------------------------------------
x = vector_face_3
v_f_face,    mean_f_face,     covar_f_face  =  ModelTrain(x)
#---------------------TRAINING MODEL with NON FACE DATA-------------------------------------------------
x = vector_nonface_3
v_f_nonface,    mean_f_nonface,     covar_f_nonface  =  ModelTrain(x)




#--------------------------------Likelihood for TEST FACE ---------------------------------------
x_t = vector_face_3_test
Prx1_w1  =  Likelihd (x_t, v_f_face, mean_f_face, covar_f_face, NumOfFeatures, 100)

x_t = vector_face_3_test
Prx1_w0  =  Likelihd (x_t, v_f_nonface, mean_f_nonface, covar_f_nonface, NumOfFeatures, 100)

Prx1_w1 = Prx1_w1.T
Prx1_w0 = Prx1_w0.T



#--------------------------------Likelihood for TEST NONFACE ---------------------------------------
x_t = vector_nonface_3_test
Prx0_w1  =  Likelihd (x_t, v_f_face, mean_f_face, covar_f_face, NumOfFeatures, 100)

x_t = vector_nonface_3_test
Prx0_w0  =  Likelihd (x_t, v_f_nonface, mean_f_nonface, covar_f_nonface, NumOfFeatures, 100)

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
    
    Post_w0_x1[0,i] = num_w0/den
    Post_w1_x1[0,i] = num_w1/den
    
    
    num_w1 = Prx0_w1[0,0,i] * Prior_w1
    num_w0 = Prx0_w0[0,0,i] * Prior_w0
    
    den = Prx0_w1[0,0,i] * Prior_w1 + Prx0_w0[0,0,i] * Prior_w0
    
    Post_w0_x0[0,i] = num_w0/den
    Post_w1_x0[0,i] = num_w1/den
    
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

Thresh = 10**(-15)

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




