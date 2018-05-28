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

import inspect

local_vars = {}


def perform_PCA(X,pca_components):
    pca = decomposition.PCA(n_components=pca_components)
    pca.fit(X)
    X_PCA = pca.transform(X)
    return X_PCA

'''#===============================Beggining of the program============================================'''

#===============Initialisations========================

Task = 2
    
Num_of_features = 75 #10800
size = int(np.sqrt(Num_of_features/3))
#data_retrieve_path = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3"
#data_store_path = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/"

vector_face_2 = np.zeros((1,10800), dtype=np.uint8)#[[0:10800]]
vector_face_2 = np.array(vector_face_2)

vector_nonface_2 = np.zeros((1,10800), dtype=np.uint8)#[[0:10800]]
vector_nonface_2 = np.array(vector_nonface_2)


#-----------------------------------training data----------------------------------------------------

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

vector_face_2 = perform_PCA(vector_face_2,Num_of_features)
vector_nonface_2 = perform_PCA(vector_nonface_2,Num_of_features)

vector_face_3 = vector_face_2.T
vector_nonface_3 = vector_nonface_2.T

if Task == 1:
    

    Mean_w1 = []#np.empty((1,1000), dtype=np.uint8)#[[0:10800]]
    Cov_w1 = []
    
    Mean_w0 = []#np.empty((1,1000), dtype=np.uint8)#[[0:10800]]
    Cov_w0 = []
    
    
    for i in range (0,Num_of_features):
        
        tmp_mean = np.mean(vector_face_3[i]) 
        Mean_w1 = np.append(Mean_w1,tmp_mean)
        
        tmp_mean = np.mean(vector_nonface_3[i]) 
        Mean_w0 = np.append(Mean_w0,tmp_mean)
        
        
        
        
        
    
    Mean_w1 = np.array([Mean_w1])    
    mean_vis_face = np.reshape(Mean_w1, (size,size,3)).astype('uint8')
    
    Mean_w0 = np.array([Mean_w0])    
    mean_vis_nonface = np.reshape(Mean_w0, (size,size,3)).astype('uint8')
    
    
    
    
    
    
    for i in range (0,Num_of_features):
        
        tmp_cov = np.cov(vector_face_3[i])  
        Cov_w1 = np.append(Cov_w1,tmp_cov)
        
        tmp_cov = np.cov(vector_nonface_3[i])  
        Cov_w0 = np.append(Cov_w0,tmp_cov)
    
    
    
    Cov_w1 = np.array(Cov_w1)    
    cov_vis_face_diag = np.diag(Cov_w1)   #Diagonalising
    
    cov_vis_face = np.sqrt(cov_vis_face_diag)
    
    cov_vis_face = np.diag(cov_vis_face)  #Returns the diagonal element
    
    cov_vis_face = np.array([cov_vis_face]) #2-D vector
    cov_vis_face = np.reshape(cov_vis_face, (size,size,3)).astype('uint8')
    
    
    Cov_w0 = np.array(Cov_w0)    
    cov_vis_nonface_diag = np.diag(Cov_w0)   #Diagonalising
    
    cov_vis_nonface = np.sqrt(cov_vis_nonface_diag)
    
    cov_vis_nonface = np.diag(cov_vis_nonface)  #Returns the diagonal element
    
    cov_vis_nonface = np.array([cov_vis_nonface]) #2-D vector
    cov_vis_nonface = np.reshape(cov_vis_nonface, (size,size,3)).astype('uint8')
    
    '''
    cv2.imshow('image2', mean_vis_face)
    cv2.imshow('image3', cov_vis_face)
    
    cv2.imshow('image4', mean_vis_nonface)
    cv2.imshow('image5', cov_vis_nonface)
    
    
    k = cv2.waitKey(0)
    cv2.destroyAllWindows()
    '''   
       
    mean_face = mean_vis_face.flatten()    
    mean_face = np.array([mean_face])
    mean_face = mean_face.T
    
        
    mean_nonface = mean_vis_nonface.flatten()    
    mean_nonface = np.array([mean_nonface])
    mean_nonface = mean_nonface.T
    
    cov_face = cov_vis_face_diag
    cov_nonface = cov_vis_nonface_diag 
    #P = []  
    #P_w0 = []   
    P_w1_x_list = [] 
    P_w0_x_list = []
    AAA = []
    
    Path = 0 # 1 --Face data; 0 = NonFace
       
    #---------------------Estimation of P(X|w=0) and P(X|w=1) using normal distrubution-----------------------
    for i in range (1000,1020):
        
        if Path == 1:
            Testdata_retrieve_path_face = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3/"+'img_' + str(i) + '.jpg'
            image_path_test = Testdata_retrieve_path_face
        elif Path == 0:
            Testdata_retrieve_path_nonface = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/nonface3/"+'img_' + str(i) + '.jpg'
            
            image_path_test = Testdata_retrieve_path_nonface
        
        new_x_w1 = cv2.imread(image_path_test, 1) 
        cv2.imshow('image2', new_x_w1)
        k = cv2.waitKey(0)
        cv2.destroyAllWindows()
        new_x_w1 = cv2.resize(new_x_w1,(size,size),interpolation = cv2.INTER_CUBIC)
        
        #new_x_w0 = cv2.imread(Testdata_retrieve_path_nonface, 1)  
        #new_x_w0 = cv2.resize(new_x_w0,(size,size),interpolation = cv2.INTER_CUBIC)
        
        
        
        
        
        
        new_x_w1 = np.array(new_x_w1)
        #new_x_w0 = np.array(new_x_w0)
        
        
        
        #new_x_w0 = new_x_w0.flatten()
        new_x_w1 = new_x_w1.flatten()
        
        
        
        new_x_w1 = np.array([new_x_w1])
        #new_x_w0 = np.array([new_x_w0])
        
        new_x_w1 = new_x_w1.T
        #new_x_w0 = new_x_w0.T
        
        
        
        fact_3 = (new_x_w1 - mean_face)
        #ZZZ = preprocessing.normalize(cov_face, norm='l2')
        fact_2 = inv(cov_face)
        fact_1 = fact_3.T
        
        fact_3_w0 = (new_x_w1 - mean_nonface)
        #ZZZ = preprocessing.normalize(cov_face, norm='l2')
        fact_2_w0 = inv(cov_nonface)
        fact_1_w0 = fact_3_w0.T
        '''
        D = cov_face.shape[0]
        const = int(np.round(2*np.pi))
        fact_const_w1 = (np.sqrt(const^D))*np.sqrt(np.linalg.det(cov_face))
        fact_const_w0 = (np.sqrt(const^D))*np.sqrt(np.linalg.det(cov_nonface)) 
        '''
        
        index = np.dot(fact_1,fact_2)
        index = -0.5*np.dot(index,fact_3)
        #P.append(np.exp(index))
        P_x_w1 =(np.exp(index))
        print(P_x_w1)
        #AAA.append(P_x_w1)
        
        index_0 = np.dot(fact_1_w0,fact_2_w0)
        index_0 = -0.5*np.dot(index_0,fact_3_w0)
        P_x_w0 = (np.exp(index_0))
        print(P_x_w0)
        
        Prior_w0 = 0.5
        Prior_w1 = (1-Prior_w0)
        
        
        Num_w1 = P_x_w1*Prior_w1
        Num_w0 = P_x_w0*Prior_w0
        Den = P_x_w1*Prior_w1+P_x_w0*Prior_w0
        
        P_w1_x = Num_w1/Den
        P_w0_x = Num_w0/Den
        
        #P_w1_x = Num_w1/Den
        if Path == 1:
            print('Positives being classified as faces: ',P_w1_x)
            print('Positives being classified as nonfaces: ',P_w0_x)
        elif Path == 0:
            print('Negatives being classified as faces: ',P_w1_x)
            print('Negatives being classified as nonfaces: ',P_w0_x)
            
            
        '''================Face Data ==============================='''    
        #Positives being classified as faces
        #Negatives being classified as faces
        P_w1_x_list.append(P_w1_x)
        
        #Positives being classified as nonfaces
        #Negatives being classified as nonfaces
        P_w0_x_list.append(P_w0_x)
        
        '''================Non Face Data ==============================='''       
    
#def Initialise_GMM(X,pca_components):
    
    
#def E_step():
        
        
def  Initialize_GMM(NumOfModels, NumOfFeatures, NumOfImages):
    
    #NumOfModels = 3 # K
    #NumOfFeatures = 75
    
    mean = np.random.rand(1,NumOfFeatures,NumOfModels)
    covar = np.random.rand(3,NumOfFeatures,NumOfFeatures)
    lmbda = np.random.rand(1,NumOfModels)
    r = np.random.rand(1,NumOfImages,NumOfModels)
    
    return mean, covar, lmbda, r
    
def E_step(Mean,Covar,Lmbda, rik): 
    global local_vars
    size = 5
    mean = Mean
    covar = Covar
    lmbda = Lmbda
    r = rik
    
    
    m=0
    Path = 1
    for i in range (1000,1004):
    
        if Path == 1:
            Testdata_retrieve_path_face = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3/"+'img_' + str(i) + '.jpg'
            image_path_test = Testdata_retrieve_path_face
        elif Path == 0:
            Testdata_retrieve_path_nonface = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/nonface3/"+'img_' + str(i) + '.jpg'
            
            image_path_test = Testdata_retrieve_path_nonface
        
        new_x_w1 = cv2.imread(image_path_test, 1) 
        #cv2.imshow('image2', new_x_w1)
        #k = cv2.waitKey(0)
        #cv2.destroyAllWindows()
        new_x_w1 = cv2.resize(new_x_w1,(size,size),interpolation = cv2.INTER_CUBIC)
        
        
        new_x_w1 = np.array(new_x_w1)
        
        new_x_w1 = new_x_w1.flatten()
        
        new_x_w1 = np.array([new_x_w1])
        
        new_x_w1 = new_x_w1.T
        
        mean1 = (np.array([mean[0,:,0]])).T
        mean2 = (np.array([mean[0,:,1]])).T
        mean3 = (np.array([mean[0,:,2]])).T
        
        covar1 = covar[0,:,:]
        covar2 = covar[1,:,:]
        covar3 = covar[2,:,:]
        
        lmbda1 = np.array([lmbda[0,0]])
        lmbda2 = np.array([lmbda[0,1]])
        lmbda3 = np.array([lmbda[0,2]])

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
        P_x_m1 =(np.exp(index_m1))
        print(P_x_m1)
        
        index_m2 = np.dot(fact_1_m2,fact_2_m2)
        index_m2 = -0.5*np.dot(index_m2,fact_3_m2)      
        P_x_m2 =(np.exp(index_m2))
        print(P_x_m2)
        
        index_m3 = np.dot(fact_1_m3,fact_2_m3)
        index_m3 = -0.5*np.dot(index_m3,fact_3_m3)      
        P_x_m3 =(np.exp(index_m3))
        print(P_x_m3)
        
        
        Prior_h1 = lmbda1
        Prior_h2 = lmbda2
        Prior_h3 = lmbda3
        
        
        
        Den = P_x_m1*Prior_h1 + P_x_m2 * Prior_h2 + P_x_m3 * Prior_h3
        
        
        P_h1_x = (P_x_m1 * Prior_h1) / Den
        r[0,m,0] = 1#P_h1_x
        
        P_h2_x = (P_x_m2 * Prior_h2) / Den
        r[0,m,1] = 2#P_h2_x
        
        P_h3_x = (P_x_m3 * Prior_h3) / Den
        r[0,m,2] = 3#P_h3_x
        
        m = m+1
    local_vars = inspect.currentframe().f_locals    
    return r


def M_step(rik):
    
    size = 5
    r = rik
    num = np.sum(r,axis = 1) # [4,8,12]   [0,modelno]
    
    den = np.sum(r)
    
    lmbda[0,0] = num[0,0]/den
    lmbda[0,1] = num[0,0]/den
    lmbda[0,2] = num[0,0]/den
    
    Path = 1
    x = np.zeros((1,Num_of_features), dtype=np.uint8)#[[0:10800]]
    for i in range (1000,1004):
    
        if Path == 1:
            Testdata_retrieve_path_face = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/3/"+'img_' + str(i) + '.jpg'
            image_path_test = Testdata_retrieve_path_face
        elif Path == 0:
            Testdata_retrieve_path_nonface = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/nonface3/"+'img_' + str(i) + '.jpg'
            
            image_path_test = Testdata_retrieve_path_nonface
        
        new_x_w1 = cv2.imread(image_path_test, 1) 
        #cv2.imshow('image2', new_x_w1)
        #k = cv2.waitKey(0)
        #cv2.destroyAllWindows()
        new_x_w1 = cv2.resize(new_x_w1,(size,size),interpolation = cv2.INTER_CUBIC)
        
        
        new_x_w1 = np.array(new_x_w1)
        
        new_x_w1 = new_x_w1.flatten()
        
        new_x_w1 = np.array([new_x_w1])
        x = np.append(x, new_x_w1, axis =0)
        
        
    x = np.delete(x,0,0) 
    r = r.T
    
    den =np.sum(r)
    
    bbb = (np.dot(r[:,:,0],x)/den)
    bbb = bbb.T
    mean = np.array([bbb])
    
    x = x.T
    #fact_1 =
    
    NumOfModels = 3
    NumOfImage  = 4                             # Change this
    
    tmp_num = np.zeros((NumOfImage,Num_of_features,Num_of_features))
    tmp_covar = np.zeros((NumOfModels,Num_of_features,Num_of_features))
    for k in range (0,NumOfModels):
        for i in range (0,NumOfImage):
            
            fact_1 = r[k,i,0]
            fact_2 = (np.array([(x[:,i] - mean[0,:,k])])).T
            fact_3 = fact_2.T
            
            num1 = np.dot(fact_2,fact_3)
            num = np.dot(fact_1,num1)
            
            tmp_num[i,:,:] = num
            
        tmp_covar[k,:,:] = np.sum(tmp_num, axis=0)
    covar = tmp_covar/den
        
    return lmbda, mean, covar
    
if Task == 2:
    
    #==================Initializing mean,  variance and hidden variable parameter============
    NumOfModels, NumOfFeatures, NumOfImages = 3, 75, 4
    Convergence = False
    first_run = 1   
    tolerance_mean = 1
    mean, covar, lmbda, r= Initialize_GMM(NumOfModels, NumOfFeatures, NumOfImages)
    
       
    while (Convergence == False):
        if first_run == 1:           
            mean_t0, covar_t0, lmbda_t0, r = mean, covar, lmbda, r
            
        r = E_step(mean,covar,lmbda, r)
    
        lmbda_t1, mean_t1, covar_t1 = M_step(r)
        
        first_run = 0
        
        if abs(lmbda_t0.all() - lmbda_t1.all()) < tolerance_mean:
            Convergence = True
        else:
            mean_t0, covar_t0, lmbda_t0 = lmbda_t1, mean_t1, covar_t1 
            
            


            
            
            
            
            
            
            
        
        
    
        
    
        
        
        
    
    
    
    
    
    
    
    
    

       
    

    
    
    
    