# -*- coding: utf-8 -*-
"""
Created on Wed Feb 21 18:30:27 2018

@author: adity
"""

import cv2
import inspect
import numpy as np
from numpy.linalg import inv
from numpy.linalg import det
from sklearn.preprocessing import StandardScaler
from scipy.stats import multivariate_normal


import matplotlib.pyplot as plt
from sklearn import metrics


from sklearn import preprocessing
from sklearn import decomposition
local_vars = {}
local_vars_Estep = {}
local_vars_Mstep = {}
local_vars_Init = {}



#--------------------------------------------Function Declaration-----------------------------------
def perform_PCA(X,pca_components):
    pca = decomposition.PCA(n_components=pca_components)
    pca.fit(X)
    X_PCA = pca.transform(X)
    return X_PCA



def  Initialize_GMM(NumOfModels, NumOfFeatures, NumOfImages):
    global local_vars_Init
    
    #NumOfModels = 3 # K
    #NumOfFeatures = 75
    
    mean            =       np.random.rand(1,NumOfFeatures,NumOfModels)
    tmp_covar       =       np.random.randint(1000,5000,(3,NumOfFeatures,1))
    covar           =       np.zeros((3,NumOfFeatures,NumOfFeatures))
    
    for i in range (0,3):
        covar[i,:,:]        =       np.diagflat(tmp_covar[i,:,:])
        
    lmbda           =       np.random.rand(1,NumOfModels)
    r               =       np.random.rand(NumOfImages,NumOfModels)
    
    local_vars_Init = inspect.currentframe().f_locals   
    
    return mean, covar, lmbda, r
    


def E_step_GMM(x, Mean,Covar,Lmbda, NoOfImages_train, NumOfModels): 
    global local_vars_Estep
    
    mean                    =           Mean
    covar                   =           Covar
    lmbda                   =           Lmbda
    #r                       =           rik
    
    
    #lik         =       np.zeros((NoOfImages_train, NumOfModels))#[[0:10800]]
    #rik         =       np.ones((NoOfImages_train, NumOfModels))#[[0:10800]]
    
    lik               =       np.random.rand(NoOfImages_train,NumOfModels)
    rik               =       np.random.rand(NoOfImages_train,NumOfModels)
    
    
    
    for i in range (0,NoOfImages_train):
        for k in range (0,NumOfModels):
            
        
            new_x_w1                =       x[:,i]
            image_nonface_new       =       (np.array([new_x_w1])).T
            
            mean_face               =       np.array([mean[0,:,k]]).T
            
            cov_face                =       covar[k,:,:]
            
                
            #y = multivariate_normal.pdf(new_x_w1, tmp_mean, tmp_covar)
            
            f3_w1           =           (image_nonface_new - mean_face)  
            f2_w1           =           inv(cov_face)
            f1_w1           =           f3_w1.T
            
            tmp_index_w1    =           np.dot(f1_w1,f2_w1)
            index_w1        =           np.dot(tmp_index_w1,f3_w1)
            
            Pr_x0_w1        =           np.exp(-0.5*index_w1)
            tmp_lmbd        =           lmbda[0,k]
            
            lik[i,k]        =           tmp_lmbd*Pr_x0_w1       #value here goes to zero
         
            
            
        tmp_lik         =           lik[i,:]
        den             =       np.sum(tmp_lik)
        
        for k in range (0,NumOfModels):
            
            rik[i,k]        =           lik[i,k]/den            #value here goes to nan as lik is zero
            
    '''for k in range (0,NumOfModels):
        
        lmbda[0,k]      =       np.sum(rik[:,k])/np.sum(rik)'''
        
        
        

    local_vars_Estep = inspect.currentframe().f_locals    
    return rik


def M_step_GMM(x, Mean, rik, NoOfImages_train, NumOfFeatures, NumOfModels):
        global local_vars_Mstep
        
        tmp_mean = Mean
        Num_of_features = NumOfFeatures
        tmp_x = x.T
        tmp_r = rik.T
        #r_m = rik
        lmbda  =  np.zeros((1,NumOfModels))
        
        
        den = np.sum(rik)
        
        for k in range (0,NumOfModels):
        
            lmbda[0,k]      =       np.sum(rik[:,k])/np.sum(rik)
            
            num             =       np.dot(tmp_r[k,:],tmp_x)
            tmp_mean[0,:,k]     =       num/den
        
        lmbda = lmbda
        mean  = tmp_mean
        
        
        
        tmp_num     =   np.random.rand(NoOfImages_train,     Num_of_features,     Num_of_features)
        tmp_covar   =   np.random.rand(  NumOfModels,    Num_of_features,    Num_of_features)
        
        for k in range (0,NumOfModels):
            for i in range (0,NoOfImages_train):
                
                fact_1 = rik[i,k]
                fact_2 = (np.array([(x[:,i] - mean[0,:,k])])).T
                fact_3 = fact_2.T
                
                num1 = np.dot(fact_2,fact_3)
                num = np.dot(fact_1,num1)
                
                tmp_num[i,:,:] = num
                
            tmp_covar[k,:,:] = np.sum(tmp_num, axis=0)
        covar = tmp_covar/den
        
        local_vars_Mstep = inspect.currentframe().f_locals    
        return mean, covar, lmbda


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



Pixels = dis_size*dis_size*channels
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
    
    #image_w1 = cv2.resize(image_w1,(down_size,down_size),interpolation = cv2.INTER_CUBIC)
    #image_w0 = cv2.resize(image_w0,(down_size,down_size),interpolation = cv2.INTER_CUBIC)
       
    
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
    
    
    #new_x_w1        =       cv2.resize(new_x_w1,(down_size,down_size),interpolation = cv2.INTER_CUBIC)
    #new_x_w0        =       cv2.resize(new_x_w0,(down_size,down_size),interpolation = cv2.INTER_CUBIC)
    
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

scalar_nonface_test =StandardScaler()
scalar_nonface_test.fit(vector_nonface_2_1_test)
vector_nonface_2_1_test = scalar_nonface_test.transform(vector_nonface_2_1_test)



#---------------------VECTOR with TESTING IMAGE DATA-----------------------------------------------
vector_face_3_test       =      (vector_face_2_1_test).T          #Dim[Features x NoOfImages]
vector_nonface_3_test    =      (vector_nonface_2_1_test).T
#---------------------------------------------------------------------------------------------------




#-----------------------------------Training the Model for FACE DATA -------------------------------

x = vector_face_3


MEAN, COVAR, LMBDA, R   =       Initialize_GMM(NumOfModels, NumOfFeatures, NoOfImages_train)
mean_t0, covar_t0, lmbda_t0, = MEAN, COVAR, LMBDA

Convergence = False    
first_run = 1 
converg_cntr  = 0   
tolerance_mean = 0.1

while (Convergence == False):
        #if first_run == 1: 
            
            
            
        #===================E_STEP=================================================    
        r = E_step_GMM(x,mean_t0, covar_t0, lmbda_t0, NoOfImages_train, NumOfModels)
        
        
        #===================M_STEP=================================================
        mean_t1, covar_t1, lmbda_t1 = M_step_GMM(x, mean_t0, r, NoOfImages_train, NumOfFeatures, NumOfModels)
        
        converg_cntr = converg_cntr +1
        print('====================Counter:',converg_cntr)
        
        first_run = 0
        
        #mean_t0, covar_t0, lmbda_t0 = mean_t1, covar_t1, lmbda_t1 
        
        
        
        if (abs(lmbda_t0[0,2] - lmbda_t1[0,2]) < tolerance_mean) or converg_cntr >= 1:
            Convergence = True
            lmbda_final,mean_final, covar_final  = lmbda_t1, mean_t1, covar_t1 
        else:
            mean_t0, covar_t0, lmbda_t0 = mean_t1, covar_t1, lmbda_t1 

lmbda_final_face,mean_final_face, covar_final_face = lmbda_final,mean_final, covar_final


#-----------------------------------Training the Model for NONFACE DATA ----------------------------

x = vector_nonface_3


MEAN, COVAR, LMBDA, R   =       Initialize_GMM(NumOfModels, NumOfFeatures, NoOfImages_train)
mean_t0, covar_t0, lmbda_t0, = MEAN, COVAR, LMBDA

Convergence = False    
first_run = 1 
converg_cntr  = 0   
tolerance_mean = 0.1

while (Convergence == False):
        #if first_run == 1: 
            
            
            
        #===================E_STEP=================================================    
        r = E_step_GMM(x,mean_t0, covar_t0, lmbda_t0, NoOfImages_train, NumOfModels)
        
        
        #===================M_STEP=================================================
        mean_t1, covar_t1, lmbda_t1 = M_step_GMM(x, mean_t0, r, NoOfImages_train, NumOfFeatures, NumOfModels)
        
        converg_cntr = converg_cntr +1
        print('====================Counter:',converg_cntr)
        
        first_run = 0
        
        mean_t0, covar_t0, lmbda_t0 = mean_t1, covar_t1, lmbda_t1 
        
        
        
        if (abs(lmbda_t0[0,2] - lmbda_t1[0,2]) < tolerance_mean) or converg_cntr >= 1:
            Convergence = True
            lmbda_final,mean_final, covar_final  = lmbda_t1, mean_t1, covar_t1 
        else:
            mean_t0, covar_t0, lmbda_t0 = mean_t1, covar_t1, lmbda_t1 
            
lmbda_final_nonface,mean_final_nonface, covar_final_nonface = lmbda_final,mean_final, covar_final





def likelihood(Lmbda, Mean, Cov):
    Prob_final = []
    for i in range (0,100):
    
    
        new_x_w1 = x[:,i]
        new_x_w1 = (np.array([new_x_w1])).T
        
        
        mean1 = (np.array([tmp_mean[0,:,0]])).T
        mean2 = (np.array([tmp_mean[0,:,1]])).T
        mean3 = (np.array([tmp_mean[0,:,2]])).T
        
        covar1 = tmp_covar[0,:,:]
        covar2 = tmp_covar[1,:,:]
        covar3 = tmp_covar[2,:,:]
        
        lmbda1 = np.array([tmp_lmbda[0,0]])
        lmbda2 = np.array([tmp_lmbda[0,1]])
        lmbda3 = np.array([tmp_lmbda[0,2]])
        
        fact_3_m1 = (new_x_w1 - mean1)
        fact_2_m1 = inv(covar1)
        fact_1_m1 = fact_3_m1.T
        
        fact_3_m2 = (new_x_w1 - mean2)
        fact_2_m2 = inv(covar2)
        fact_1_m2 = fact_3_m2.T
        
        fact_3_m3 = (new_x_w1 - mean3)
        fact_2_m3 = inv(covar3)
        fact_1_m3 = fact_3_m3.T
        
        
        index_m1 = np.dot(fact_1_m1,fact_2_m1)
        index_m1 = -0.5*np.dot(index_m1,fact_3_m1)      
        P_x_m1 =(np.exp(index_m1))                      #Norm_m1
        #print('P_x_m1=',P_x_m1)
        
        index_m2 = np.dot(fact_1_m2,fact_2_m2)
        index_m2 = -0.5*np.dot(index_m2,fact_3_m2)      
        P_x_m2 =(np.exp(index_m2))
        #print('P_x_m2=',P_x_m2)
        
        index_m3 = np.dot(fact_1_m3,fact_2_m3)
        index_m3 = -0.5*np.dot(index_m3,fact_3_m3)      
        P_x_m3 =(np.exp(index_m3))
        #print('P_x_m3=',P_x_m3)
        
        
        Prior_h1 = lmbda1
        Prior_h2 = lmbda2
        Prior_h3 = lmbda3
        
        
        
        Prob = P_x_m1[0,0]*Prior_h1 + P_x_m2[0,0] * Prior_h2 + P_x_m3[0,0] * Prior_h3
        
        Prob_final.append(Prob)    
    Prob_final = np.array(Prob_final)
    return Prob_final






#----------------------------------------Likelihood face Data---------------------------------------
x = vector_face_3_test

tmp_lmbda = lmbda_final_face
tmp_covar = covar_final_face
tmp_mean = mean_final_face



Prx1_w1_f = likelihood(tmp_lmbda, tmp_mean, tmp_covar)
    
tmp_lmbda = lmbda_final_nonface
tmp_covar = covar_final_nonface
tmp_mean = mean_final_nonface    

Prx1_w0_f = likelihood(tmp_lmbda, tmp_mean, tmp_covar)
    
#----------------------------------------Likelihood face Data---------------------------------------

x = vector_nonface_3_test


tmp_lmbda = lmbda_final_face
tmp_covar = covar_final_face
tmp_mean = mean_final_face


Prx0_w1_f = likelihood(tmp_lmbda, tmp_mean, tmp_covar)
    
tmp_lmbda = lmbda_final_nonface
tmp_covar = covar_final_nonface
tmp_mean = mean_final_nonface    

Prx0_w0_f = likelihood(tmp_lmbda, tmp_mean, tmp_covar)


#----------------------------Code for posterior P_w_x------------------------------
Post_w0_x1  = np.zeros((1,100))
Post_w1_x1  = np.zeros((1,100))

Post_w0_x0  = np.zeros((1,100))
Post_w1_x0  = np.zeros((1,100))
Prior_w1 = 0.5
Prior_w0 = 0.5

True_Neg    =   []#np.zeros((100,1))
False_Neg    =  []# np.zeros((100,1))

for i in range (0,100):
    
    num_w1 = Prx1_w1_f[i,0] * Prior_w1
    num_w0 = Prx1_w0_f[i,0] * Prior_w0
    
    den = Prx1_w1_f[i,0] * Prior_w1 + Prx1_w0_f[i,0] * Prior_w0
    
    Post_w0_x1[0,i] = num_w0/den        #------------face - nonface model
    Post_w1_x1[0,i] = num_w1/den        #------------face - face model
    
    
    num_w1 = Prx0_w1_f[i,0] * Prior_w1
    num_w0 = Prx0_w0_f[i,0] * Prior_w0
    
    den = Prx0_w1_f[i,0] * Prior_w1 + Prx0_w0_f[i,0] * Prior_w0
    
    Post_w0_x0[0,i] = num_w0/den        #------------nonface - nonface model
    Post_w1_x0[0,i] = num_w1/den        #------------nonface - face model



    True_Neg.append(Post_w0_x1[0,i])
    False_Neg.append(Post_w1_x0[0,i])
    
True_Neg    = np.array(True_Neg)
False_Neg    = np.array(False_Neg)

Posterior = np.append(True_Neg,False_Neg)
labels = np.append(np.ones(100), np.zeros(100))

fpr, tpr, _ = metrics.roc_curve(labels,Posterior, pos_label=0)
plt.plot(fpr, tpr, color='darkorange')
plt.xlim([0,1])
plt.ylim([0,1])
plt.show()






'''

tmp_lmbda = lmbda_final_face
tmp_covar = covar_final_face
tmp_mean = mean_final_face
Prob_final_nonface = []
x = vector_nonface_3_test

tmp_lmbda = lmbda_final_face
tmp_covar = covar_final_face
tmp_mean = mean_final_face
    
for i in range (0,100):
    
    
    new_x_w1 = x[:,i]
    new_x_w1 = (np.array([new_x_w1])).T
    
    
    mean1 = (np.array([tmp_mean[0,:,0]])).T
    mean2 = (np.array([tmp_mean[0,:,1]])).T
    mean3 = (np.array([tmp_mean[0,:,2]])).T
    
    covar1 = tmp_covar[0,:,:]
    covar2 = tmp_covar[1,:,:]
    covar3 = tmp_covar[2,:,:]
    
    lmbda1 = np.array([tmp_lmbda[0,0]])
    lmbda2 = np.array([tmp_lmbda[0,1]])
    lmbda3 = np.array([tmp_lmbda[0,2]])
    
    fact_3_m1 = (new_x_w1 - mean1)
    fact_2_m1 = inv(covar1)
    fact_1_m1 = fact_3_m1.T
    
    fact_3_m2 = (new_x_w1 - mean2)
    fact_2_m2 = inv(covar2)
    fact_1_m2 = fact_3_m2.T
    
    fact_3_m3 = (new_x_w1 - mean3)
    fact_2_m3 = inv(covar3)
    fact_1_m3 = fact_3_m3.T
    
    
    index_m1 = np.dot(fact_1_m1,fact_2_m1)
    index_m1 = -0.5*np.dot(index_m1,fact_3_m1)      
    P_x_m1 =(np.exp(index_m1))                      #Norm_m1
    #print('P_x_m1=',P_x_m1)
    
    index_m2 = np.dot(fact_1_m2,fact_2_m2)
    index_m2 = -0.5*np.dot(index_m2,fact_3_m2)      
    P_x_m2 =(np.exp(index_m2))
    #print('P_x_m2=',P_x_m2)
    
    index_m3 = np.dot(fact_1_m3,fact_2_m3)
    index_m3 = -0.5*np.dot(index_m3,fact_3_m3)      
    P_x_m3 =(np.exp(index_m3))
    #print('P_x_m3=',P_x_m3)
    
    
    Prior_h1 = lmbda1
    Prior_h2 = lmbda2
    Prior_h3 = lmbda3
    
    
    
    Prob = P_x_m1[0,0]*Prior_h1 + P_x_m2[0,0] * Prior_h2 + P_x_m3[0,0] * Prior_h3
    
    Prob_final_nonface.append(Prob)     
    
Prob_final_face = np.array([Prob_final_face])
Prob_final_nonface = np.array([Prob_final_nonface])

Poscntr = 0
for i in range (0,100):
    if Prob_final_face[0,i] > Prob_final_nonface[0,i]:
        Poscntr = Poscntr + 1
        
Accuracy = Poscntr/100
'''       