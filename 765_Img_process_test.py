# -*- coding: utf-8 -*-
"""
Created on Thu Nov 23 00:10:17 2017

@author: adity
"""

import numpy as np
import cv2



#==============================DATA ACQUISITION================================

#==============================EDGE PROBABILITY MATRIX=========================
img = cv2.imread('001_sample.jpg')
#cv2.imshow("img",img)

#==============================TRAINING LABELS=================================
label = cv2.imread('001_sample_label.jpg')
#cv2.imshow("label",label)


gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#cv2.imshow("gray",gray)


ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

thresh = cv2.bitwise_not(thresh)
#cv2.imshow("thresh",thresh)

#=======================Morphological filtering================================




#=======================Kernel definition======================================
kernel = np.ones((3,3),np.uint8)







'====================================== DOG =================================='

'=========================Gaussian Blur ======================================'
img_blur_Gaus1 = cv2.GaussianBlur(thresh,(0,0),2.5)
img_blur_Gaus2 = cv2.GaussianBlur(thresh,(0,0),4)
img_blur_Gaus3 = cv2.GaussianBlur(thresh,(0,0),6)
    

'=========================Difference    ======================================'
img_DOG1 = img_blur_Gaus1 - img_blur_Gaus2
img_DOG2 = img_blur_Gaus2 - img_blur_Gaus3
img_DOG3 = img_blur_Gaus1 - img_blur_Gaus3
    
cv2.imshow("DOG1",img_DOG1)










#=======================Opening================================================
#========Removal of background noise followed by dilation======================
#========Less iterations sharp images==========================================
#==============================================================================
img_opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 1)
cv2.imshow("img_opening",img_opening)

img_opening_invert = cv2.bitwise_not(img_opening)
cv2.imshow("img_opening_invert",img_opening_invert)





#=======================CLOSING================================================
#========Removal of foreground noise followed by erosion======================
#========Less iterations sharp images==========================================
#==============================================================================
img_closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel,iterations = 1)

cv2.imshow("img_closing",img_closing)

img_closing_invert = cv2.bitwise_not(img_closing)
#cv2.imshow("img_closing_invert",img_closing_invert)


#=======================Erosion================================================
#========Removal of foreground noise followed by erosion======================
#========Less iterations sharp images==========================================
#==============================================================================
img_erosion = cv2.erode(thresh,kernel,iterations = 1)

cv2.imshow("img_erosion",img_erosion)

img_erosion_invert = cv2.bitwise_not(img_erosion)
#cv2.imshow("img_erosion_invert",img_erosion_invert)


#=======================Dilation================================================
#========Removal of foreground noise followed by erosion======================
#========Less iterations sharp images==========================================
#==============================================================================
img_dilation = cv2.dilate(thresh,kernel,iterations = 1)

cv2.imshow("img_dilation",img_dilation)

img_dilation_invert = cv2.bitwise_not(img_dilation)
#cv2.imshow("img_dilation_invert",img_dilation_invert)





#======================Processing==============================================

img_pro_1 = img_opening_invert - img_closing_invert 

img_pro_1_invert = cv2.bitwise_not(img_pro_1)
#cv2.imshow("img_pro_1_invert",img_pro_1)




#=============================SKELETONIZATION==================================

 
#gray = cv2.imread('sofsk.png',0)
img = gray
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
 
cv2.imshow("skel",skel)

#=======================Opening================================================
#========Removal of background noise followed by dilation======================
#========Less iterations sharp images==========================================
#==============================================================================
img_opening_1 = cv2.morphologyEx(thresh,cv2.MORPH_CLOSE,kernel, iterations = 1)
cv2.imshow("img_opening_1",img_opening_1)

img_opening_1_invert = cv2.bitwise_not(img_opening_1)
cv2.imshow("img_opening_1_invert",img_opening_1_invert)

#==================Bilateral Blurring==========================================
img_bilateral = cv2.bilateralFilter(img_opening_1,9,75,75)
cv2.imshow("img_bilateral",img_bilateral)





print('Done')





#cv2.imshow("sure_bg",sure_bg)
#cv2.imshow("sure_fg",sure_fg)
#cv2.imshow("!!!",DT)
cv2.waitKey(0)
cv2.destroyAllWindows()
