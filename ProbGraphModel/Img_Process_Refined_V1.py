# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 01:59:25 2017

@author: aditya
"""



import cv2
import numpy as np
from matplotlib import pyplot as plt

'====================EPM======================================================='
Sample_Image = '020_sample.jpg'

img1111 = cv2.imread(Sample_Image)
cv2.imshow("EPM",img1111)

'====================label======================================================='
img_label = cv2.imread(Sample_Image)
cv2.imshow("Label",img_label)

'====================Gray======================================================='
img = cv2.cvtColor(img1111,cv2.COLOR_BGR2GRAY)
cv2.imshow("GS",img)



    
def  DistanceTransform(Image,Window):
    dist_transform = cv2.distanceTransform(Image,cv2.DIST_C,3)
    cv2.imshow("Distance_transform_"+str(Window),dist_transform)
    return dist_transform

def MedianBlur(Image,Window):
    img_median = cv2.medianBlur(Image,3)
    #cv2.imshow('2',img_median)
    cv2.imshow("Med_Blur_"+str(Window),img_median)
    return img_median 
    

def Threshold(Image,Window):
    ret2,img_thd = cv2.threshold(Image,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    cv2.imshow("Thresh_"+str(Window),img_thd)
    return img_thd

def Dilation(Image,Window):
    kernel = np.ones((3,3),np.uint8)
    img_dilation = cv2.dilate(Image,kernel,iterations = 1)
    cv2.imshow("Dilation_"+str(Window),img_dilation)
    return img_dilation

def Erosion(Image,Window):
    kernel = np.ones((3,3),np.uint8)
    img_erosion = cv2.erode(Image,kernel,iterations = 1)
    cv2.imshow("Erosion_"+str(Window),img_erosion)
    return img_erosion
    
def DOG(Image,Window):
    img_blur_Gaus1 = cv2.GaussianBlur(Image,(0,0),2.5) #2.5
    img_blur_Gaus2 = cv2.GaussianBlur(Image,(0,0),4)#3.25)
    img_DOG = img_blur_Gaus1 - img_blur_Gaus2
    cv2.imshow("DOG_"+str(Window),img_DOG)
    return img_DOG

def Bilateral(Image,Window):
    img_bilateral = cv2.bilateralFilter(Image,9,75,75)
    cv2.imshow("Bilateral_"+str(Window),img_bilateral)
    return img_bilateral

def Closing(Image,Window):
    kernel = np.ones((3,3),np.uint8)
    img_closing = cv2.morphologyEx(Image, cv2.MORPH_CLOSE, kernel,iterations = 1)
    cv2.imshow("Closing_"+str(Window),img_closing)
    return img_closing

def Opening(Image,Window):
    kernel = np.ones((3,3),np.uint8)
    img_opening = cv2.morphologyEx(Image,cv2.MORPH_OPEN,kernel, iterations = 1)
    cv2.imshow("Opening_"+str(Window),img_opening)
    return img_opening

def Invert(Image,Window):
    img_invert = cv2.bitwise_not(Image)
    cv2.imshow("Invert_"+str(Window),img_invert)
    return img_invert


def Skeletonize(Image,Window):
    img = Image
    size = np.size(img)
    skel = np.zeros(img.shape,np.uint8)
     
    #ret,img = cv2.threshold(img,127,255,0) #Image,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU ----- original
    ret,img = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU) #Image,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    done = False
     
    while( not done):
        eroded = cv2.erode(img,element)
        temp = cv2.dilate(eroded,element)
        temp = cv2.subtract(img,temp)
        skel = cv2.bitwise_or(skel,temp)
        img = eroded.copy()
     
        zeros = size - cv2.countNonZero(img)
        print('zeros',zeros)
        
        if zeros==size:
            done = True
    
    #img1 = cv2.cvtColor(skel,cv2.COLOR_BGR2GRAY)
    ret2,th3 = cv2.threshold(skel,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    
    cv2.imshow("Skeleton_"+str(Window),skel)
    cv2.imshow("Seleton_th3"+str(Window),th3)
    return skel,th3







'''

img2 = MedianBlur(img,1)
img_bifil_1 =Bilateral(img2,1)
img_thres_2 = Threshold(img_bifil_1,1)




img_bifil_2 =Bilateral(img,2)
img3 = MedianBlur(img_bifil_2,2)
img_thres_3 = Threshold(img3,2)



img_bifil_3 =Bilateral(img,3)
img_thres_3 = Threshold(img_bifil_3,3)
img4 = MedianBlur(img_thres_3,3)


'''
###########################################################

img_thresh = Threshold(img,1)
img_medblu_pre = MedianBlur(img_thresh,'pre')

img_open_pre = Opening(img_medblu_pre,'pre')
img_medblu_pre2 = MedianBlur(img_open_pre,'pre_2')

img_DOG_1 = DOG(img_medblu_pre2,1)



img_eros_post = Erosion(img_DOG_1,'post')

img_skel_post, img_skel_thresh  = Skeletonize(img_eros_post,1)

##################################################

                                                                                #img_bilfil_1 = Bilateral(img_skel_post,1)
img_dil_post = Dilation(img_skel_post, 'post')

img_opening_post = Opening(img_dil_post,'post')

imgAND = cv2.bitwise_xor(img_skel_post,img_opening_post)
cv2.imshow('and',imgAND)



img_medblu_post = MedianBlur(imgAND,'post')

img_medblu_post2 = MedianBlur(img_medblu_post,'post2')


'''
1. Perform Thresholding on grayscale image
2. Perform MedianBlur filtering to remove noise
3. Perform DOG 
4. Perform Median filtering
5. Perform Skeletonization

'''
''' Case1
img1 = Threshold(img,1)
img11 =Bilateral(img1,1)

img2 = MedianBlur(img11,1)
img3 = DOG(img2,1)

img4 = MedianBlur(img3,2)
img5,img6 = Skeletonize(img4,1)
'''

''' Case2
1. Perform Thresholding on grayscale image
2. Perform MedianBlur filtering to remove noise
3. Perform DOG 
4. Perform Median filtering
5. Perform Skeletonization
'''
'''
imgd = cv2.Laplacian(img,cv2.CV_8UC1)
#imgd = cv2.Sobel(img,cv2.CV_8UC1,1,1,ksize=5)#(img, -1, 1,1,3,cv2.BORDER_ISOLATED)
cv2.imshow('q',imgd)
'''


'''

img222 = Dilation(img,2)
imgddd = Opening(img222,2)

img111 = Closing(img,1)
imgeee = Erosion(img,1)

img1 = Threshold(imgeee,1)
imgt2 = Threshold(imgddd,2)

imgeer = Dilation(imgt2,5)
imginv = Invert(imgeer,1)


img11 =Bilateral(imginv,1)

img2 = MedianBlur(img1,1)
'''

'''
imgAND = cv2.bitwise_and(img11,img2)
cv2.imshow('and',imgAND)
img3 = DOG(img2,1)

dst = cv2.addWeighted(img1,0.7,img2,0.3,0)

img4 = MedianBlur(img3,2)
'''


'''
img5,img6 = Skeletonize(img11,1)

kkk = cv2.bitwise_xor(img6,imgeer)
cv2.imshow('k',kkk)
JJJ = cv2.bitwise_not(kkk)
cv2.imshow('J',JJJ)
BBB = Bilateral(kkk,5)

'''

'''
DDD =  Dilation(kkk,5)

OOO = Opening(DDD,8)

MMMM = MedianBlur(DDD,8)

ERRR =  Erosion(MMMM,8)

MMMM = MedianBlur(ERRR,9)

ERRR =  Erosion(MMMM,9)

MMMM = MedianBlur(ERRR,10)

ERRR =  Erosion(MMMM,10)

SKKK = Skeletonize(DDD,8)
'''

cv2.waitKey(0)
cv2.destroyAllWindows()
