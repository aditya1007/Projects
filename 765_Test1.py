# -*- coding: utf-8 -*-
"""
Created on Thu Nov 23 01:23:25 2017

@author: adity
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt


img1111 = cv2.imread('001_sample.jpg')
cv2.imshow("EPM",img1111)
img_label = cv2.imread('001_sample_label.jpg')
cv2.imshow("Label",img_label)

img = cv2.cvtColor(img1111,cv2.COLOR_BGR2GRAY)
cv2.imshow("GS",img)

img_median1 = cv2.medianBlur(img,3)

ret2,th2 = cv2.threshold(img_median1,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

kernel = np.ones((3,3),np.uint8)

img_dilation = cv2.dilate(th2,kernel,iterations = 1)
cv2.imshow("img_dilation",img_dilation)





'====================================== DOG =================================='

'=========================Gaussian Blur ======================================'

'=========================Difference    ======================================'

img_blur_Gaus1 = cv2.GaussianBlur(img_dilation,(0,0),2.5)
img_blur_Gaus2 = cv2.GaussianBlur(img_dilation,(0,0),10)
img_blur_Gaus3 = cv2.GaussianBlur(img_dilation,(0,0),6)
'=========================Difference    ======================================'
img_DOG1 = img_blur_Gaus1 - img_blur_Gaus2
img_DOG2 = img_blur_Gaus1 - img_blur_Gaus3

hist = np.histogram(img_DOG1.ravel(),256,[0,256])

#cv2.imshow("hist",hist)

#plt.hist(img_DOG1.ravel(),256,[0,256]); plt.show()
    
cv2.imshow("DOG1",img_DOG1)
cv2.imshow("DOG2",img_DOG2)




img_dilation = cv2.dilate(img_DOG1,kernel,iterations = 1)
cv2.imshow("img_dilation",img_dilation)

img_median = cv2.medianBlur(img_DOG1,3)#(th3,9,75,75)
#img_bilateral = cv2.bitwise_not(img_bilateral)
cv2.imshow("img_median",img_median)

img_closing = cv2.morphologyEx(img_median, cv2.MORPH_CLOSE, kernel,iterations = 1)

cv2.imshow("img_closing",img_closing)

img_opening = cv2.morphologyEx(img_median,cv2.MORPH_OPEN,kernel, iterations = 1)
cv2.imshow("img_opening",img_opening)

img_opening_invert = cv2.bitwise_not(img_opening)
cv2.imshow("img_opening_invert",img_opening_invert)

img = img_median1
size = np.size(img)
skel = np.zeros(img.shape,np.uint8)
 
ret,img = cv2.threshold(img,127,255,0)
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

cv2.imshow("skel",skel)
cv2.imshow("th3",th3)

#cv2.medianBlur()

dist_transform = cv2.distanceTransform(img_median,cv2.DIST_C,3)
#dist_transform_inv = cv2.bitwise_not(img_median)

img_erosion = cv2.erode(dist_transform,kernel,iterations = 6)
dist_transform_inv = cv2.bitwise_not(img_erosion)


cv2.imshow("img_erosion",img_erosion)

cv2.imshow("dist_transform",dist_transform_inv)


image, contours, hierarchy = cv2.findContours(img_median,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
image, contours1, hierarchy = cv2.findContours(skel,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
image, contours2, hierarchy = cv2.findContours(img_opening,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


Chambers  = len(contours) - 1 
Chambers1 = len(contours1) - 1
Chambers2  = len(contours2) - 1
print(Chambers,Chambers1,Chambers2)




cv2.waitKey(0)
cv2.destroyAllWindows()



