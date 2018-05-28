# -*- coding: utf-8 -*-
"""
Created on Sun Mar  4 01:45:17 2018

@author: adity
"""

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
from scipy import special
import scipy
from math import * 

import matplotlib.pyplot as plt
from sklearn import metrics

from sklearn import preprocessing
from sklearn import decomposition
local_vars = {}
local_vars_E_step_t_dist = {}
local_vars_M_step_t_dist = {}
local_vars_Init = {}
local_vars_Init_Likelihood  =  {}
local_vars_Model_Train  = {}

#--------------------------------------------Function Declaration-----------------------------------
def perform_PCA(X,pca_components):
    pca = decomposition.PCA(n_components=pca_components)
    pca.fit(X)
    X_PCA = pca.transform(X)
    return X_PCA


def  Initialize_t_dist(NumOfModels, NumOfFeatures, NumOfImages):
    global local_vars_Init
        
    #NumOfModels = 3 # K
    #NumOfFeatures = 75
    
    mean            =       np.random.randint(0,255,(1,NumOfFeatures,NumOfModels))
    tmp_covar       =       np.random.randint(1000,5000,(3,NumOfFeatures,1))
    covar           =       np.zeros((NumOfModels,NumOfFeatures,NumOfFeatures))
    
    for i in range (0,3):
        covar[i,:,:]        =       np.diagflat(tmp_covar[i,:,:])
        
    pi              =       np.random.rand(1,NumOfModels)
    z               =       np.random.rand(NumOfImages,NumOfModels)
   
    v               =       np.random.randint(4,8,(1,NumOfModels))
    
    
    local_vars_Init    =       inspect.currentframe().f_locals     
    return mean, covar, v, pi#, z



def Likelihood(img_x, Mean, Covar, V, NumOfImages):
    
    global local_vars_Init_Likelihood
    x = img_x
    D = Covar.shape[0]
    #Covar = Covar * 10
    I =     NumOfImages
    num         =           scipy.special.gamma(0.5*(V+D))
    print(Covar.shape)
    deter         =           det(Covar)
    det_cov     =           np.sqrt(deter)
    den         =           ((V*np.pi)**(D*0.5))*det_cov*special.gamma(V/2)
    
    Lkhd        =           []
    Delta       =           []
    for i in range (0,I):
    
        new_x_w1                =       x[:,i]
        image_new               =       (np.array([new_x_w1])).T
        
        mean_face               =       np.array([Mean[0,:]]).T
        
        cov_face                =       Covar[:,:]
        
            
        #y = multivariate_normal.pdf(new_x_w1, tmp_mean, tmp_covar)
        
        f3_w1           =           (image_new - mean_face)  
        f2_w1           =           inv(cov_face)
        f1_w1           =           f3_w1.T
        
        tmp_index_w1    =           np.dot(f1_w1,f2_w1)
        delta        =           np.dot(tmp_index_w1,f3_w1)
        
        f4              =           (1  +  delta/V)**(-(V+D/2))
        
        lkhd            =           num*f4/den
        
        Lkhd.append(lkhd)
        Delta.append(delta)
    
    Lkhd = np.array(Lkhd)
    Delta = np.array(Delta)
    
    Lkhd = Lkhd[:,:,0]
    Delta = Delta[:,:,0]
    
    local_vars_Init_Likelihood = inspect.currentframe().f_locals   
    return Lkhd, Delta





        
def E_step_t_dist(img_x, Mean, Covar, V, Lmbda, NumOfFeatures , NumOfImages, NumOfModels):
    global local_vars_E_step_t_dist
    
    x           =       img_x
    D           =       NumOfFeatures
    I           =       NumOfImages
    
    
    mean        =           Mean
    covar       =           Covar
    lmbda       =           Lmbda
    v           =           V
    
    
    lik               =       np.random.rand(NoOfImages_train,NumOfModels)
    rik               =       np.random.rand(NoOfImages_train,NumOfModels)
    
    Uik               =       np.random.rand(NoOfImages_train,NumOfModels)
     
    
    
    Pr              =       np.zeros((NoOfImages_train,1))
    Pr              =       np.array([Pr])
    Pr              =       Pr[0,:,:]
    
    
    Delta              =       np.zeros((NoOfImages_train,1))
    Delta              =       np.array([Delta])
    Delta              =       Delta[0,:,:]
    
    print(Pr.shape)       
    
    for k in range (0,NumOfModels):
        
        mean_tmp                =       np.array([mean[0,:,k]]).T
        
        cov_tmp                 =       covar[k,:,:]
        V                       =       v[0,k]
        
        tmp_pr, tmp_delta                 =       Likelihood(x, mean_tmp, cov_tmp, V, I)
        
        
        
        Pr = np.concatenate((Pr, tmp_pr), axis = 1)
        Delta = np.concatenate((Delta,tmp_delta ), axis = 1)
        
       
        
    
    Pr = np.delete(Pr, 0, 1)
    Delta = np.delete(Delta, 0, 1)
    
        
    for i in range (0,I):
        for k in range (0,k):
            lik[i,k]  =  lmbda[0,k] * Pr[i,k]
       
        tmp_lik         =           lik[i,:]
        den             =       np.sum(tmp_lik)  
        
        Uik[i,k]        =       (v[0,k]+D)/(v[0,k]+Delta[i,k])
        
            
        
        
        for k in range (0,NumOfModels):
            
            rik[i,k]        =           lik[i,k]/den            #value here goes to nan as lik is zero
    
    
    zik       =    rik
    
    
#####################################MAXIMIZATION######################################################   
    
    
    #---------------------------lambda---------------------------------------
    lmbda_new  =  np.zeros((1,NumOfModels))
    
    
    
    lmbda_new   =  np.sum(zik, axis=0)
    lmbda_new   =  lmbda_new/I
    
    
    #--------------------------------------mean---------------------------------------------
    num = []
    den = np.zeros((NumOfModels,I))
    num = np.zeros((I, D))
    
    for i in range (0,I):
        for k in range (0,NumOfModels ):
            
            tmp_fact =  Uik.T
            fact1 = np.dot(zik[:,k],tmp_fact[k,:])
            den[k,i] = fact1
            
            tmp_x = x.T
            
            fact2 = np.dot(fact1, tmp_x[i,:])
            num[i,:] = fact2
        
        
    den = np.sum(den,axis=1)
    num = np.sum(num,axis=0)    
    
    
    mean_new            =       np.zeros((1,NumOfFeatures,NumOfModels))    
        #num = 0 + fact2
        #den = 0 + fact1
    for k in range (0,NumOfModels):
        mean_new[0,:,k] = num/den[k]
    
    #--------------------------------------covar--------------------------------------------
    fact2 = np.zeros((I,NumOfModels, D,D))
    fact1 = np.zeros((NumOfModels,I))
    num = np.zeros((NumOfModels,D, D))
    for k in range (0,NumOfModels ):
        for i in range (0,I):
        
            tmp_fact =  Uik.T
            fact1[k,i] = np.dot(zik[i,k],tmp_fact[k,i])
            #den[k,i] = fact1
            
            tmp_x = x.T
            
            tmp_fact1 = tmp_x[i,:] - mean_new[0,:,k]
            tmp_fact1 = np.array([tmp_fact1])
            tmp_fact2 = tmp_fact1.T
            
            fact2[i,k,:,:] = np.dot(tmp_fact1, tmp_fact2)
            
            fact2[i,k,:,:]  =  fact1[k,i] * fact2[i,k,:,:]
            
    fact2 = np.array(fact2)
    #fact1 = np.array(fact1)
    
    
    num_sum = np.sum(fact2, axis=0)
    #num_sum2 = np.sum(fact1, axis =1)
        
    
    den = np.sum(zik,axis=0)
    
       
        
    covar_new        =      np.zeros((NumOfModels,D, D))
    for k in range (0,NumOfModels):
        covar_new[k,:,:] = num_sum[k,:,:]/den[k]
    
    
    v_new   =  v
    
        
    
    
    
    
        
    local_vars_E_step_t_dist        =       inspect.currentframe().f_locals       
    return lmbda_new,    mean_new, covar_new, v_new





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
#K  = 2
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







def ModelTrain(img_x):
    global local_vars_Model_Train
    Mean, Covar, V, Lmbda = Initialize_t_dist(NumOfModels, NumOfFeatures, NoOfImages_train)
    
    #c_cov = Covar
    
    for i in range (0,0):
        #Covar = Covar*10 
        i_lmbda, i_mean, i_cov, i_v = E_step_t_dist(x, Mean, Covar, V, Lmbda,  NumOfFeatures , NoOfImages_train, NumOfModels)
        print('run')
        Covar = i_cov + np.identity(75)
        Lmbda   = i_lmbda
        Mean    = i_mean 
        V       = i_v
    local_vars_Model_Train = inspect.currentframe().f_locals
    return Lmbda, Covar, Mean, V


x = vector_face_3
lmbda_f_face,      covar_f_face,      mean_f_face,      v_f_face    =  ModelTrain(x)

x = vector_nonface_3
lmbda_f_nonface,      covar_f_nonface,      mean_f_nonface,      v_f_nonface   =  ModelTrain(x)


#--------------------------------Likelihood for TEST FACE ---------------------------------------
x_t = vector_face_3_test
Prx1_w1 = []
for k in range (0,NumOfModels):
    mean_tmp                =       np.array([mean_f_face[0,:,k]]).T
    cov_tmp                 =       covar_f_face[k,:,:]
    V                       =       v_f_face[0,k]
    tmp_Prx1_w1, tmp_delta             =       Likelihood (x_t, mean_tmp, cov_tmp,V, 100)
    print(k)
    Prx1_w1.append(tmp_Prx1_w1)

Prx1_w1  =  np.array(Prx1_w1)

 
Prx1_w0 = []
for k in range (0,NumOfModels):
    mean_tmp                =       np.array([mean_f_nonface[0,:,k]]).T
    cov_tmp                 =       covar_f_nonface[k,:,:]
    V                       =       v_f_nonface[0,k]
    tmp_Prx1_w0, tmp_delta             =       Likelihood (x_t, mean_tmp, cov_tmp,V, 100)
    print(k)
    Prx1_w0.append(tmp_Prx1_w0)

Prx1_w0  =  np.array(Prx1_w0)



#--------------------------------Likelihood for TEST FACE ---------------------------------------
x_t = vector_nonface_3_test
Prx0_w1 = []
for k in range (0,NumOfModels):
    mean_tmp                =       np.array([mean_f_face[0,:,k]]).T
    cov_tmp                 =       covar_f_face[k,:,:]
    V                       =       v_f_face[0,k]
    tmp_Prx0_w1, tmp_delta             =       Likelihood (x_t, mean_tmp, cov_tmp,V, 100)
    print(k)
    Prx0_w1.append(tmp_Prx0_w1)

Prx0_w1  =  np.array(Prx0_w1)

 
Prx0_w0 = []
for k in range (0,NumOfModels):
    mean_tmp                =       np.array([mean_f_nonface[0,:,k]]).T
    cov_tmp                 =       covar_f_nonface[k,:,:]
    V                       =       v_f_nonface[0,k]
    tmp_Prx0_w0, tmp_delta             =       Likelihood (x_t, mean_tmp, cov_tmp,V, 100)
    print(k)
    Prx0_w0.append(tmp_Prx0_w0)

Prx0_w0  =  np.array(Prx0_w0)



#------------------------------Face/FAce-----------------------------------------------------------
Prx1_w1_f   =   []
Prx0_w1_f   =   []
Prx1_w0_f   =   []
Prx0_w0_f   =   []

for i in range(0, 100):
    tmp_p1 =0
    tmp_p2 =0
    tmp_p3 =0
    tmp_p4 =0
    
    for k in range (0,NumOfModels):
        
        tmp_p1 = tmp_p1 + Prx1_w1[k,i,:]*lmbda_f_face[:,k]
        tmp_p2 = tmp_p2 + Prx0_w1[k,i,:]*lmbda_f_face[:,k]
        
        tmp_p3 = tmp_p1 + Prx1_w0[k,i,:]*lmbda_f_nonface[:,k]
        tmp_p4 = tmp_p2 + Prx0_w0[k,i,:]*lmbda_f_nonface[:,k]
    
    
    Prx1_w1_f.append(tmp_p1) 
    Prx0_w1_f.append(tmp_p2)
    Prx1_w0_f.append(tmp_p3)
    Prx0_w0_f.append(tmp_p4)

Prx1_w1_f   =   np.array(Prx1_w1_f)
Prx0_w1_f   =   np.array(Prx0_w1_f)
Prx1_w0_f   =   np.array(Prx1_w0_f)
Prx0_w0_f   =   np.array(Prx0_w0_f)

#----------------------------Code for posterior P_w_x------------------------------
Post_w0_x1  = np.zeros((1,100))
Post_w1_x1  = np.zeros((1,100))

Post_w0_x0  = np.zeros((1,100))
Post_w1_x0  = np.zeros((1,100))
Prior_w1 = 0.5
Prior_w0 = 0.5

True_Neg    =   []#np.zeros((100,1))
False_Neg    =  []# np.zeros((100,1))

True_Neg_1    =   []#np.zeros((100,1))
False_Neg_1    =  []# np.zeros((100,1))

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



 

'''
Poscntr = 0
for i in range (0,100):
    
    if Prx1_w1_f[i,0] > Prx0_w1_f[i,0]:
        Poscntr = Poscntr +1
        

print(Poscntr)



Negcntr = 0
for i in range (0,100):
    
    if Prx0_w0_f[i,0] > Prx0_w1_f[i,0]:
        Negcntr = Negcntr +1
        

print(Negcntr)

'''




