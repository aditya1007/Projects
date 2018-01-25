# -*- coding: utf-8 -*-
"""
Created on Fri Nov 24 22:34:02 2017

@author: aditya
"""

import cv2
import numpy as np
from sklearn import linear_model
from sklearn.cluster import KMeans
from matplotlib import pyplot as plt
from sklearn import linear_model


from scipy.spatial import distance
import math



#==============================================================================



def SingleViewGeom(Image, P1, P2, P3, P4, P5, P6, P7, X_ref, Y_ref, Z_ref, W):
    import cv2
    import numpy as np
    import numpy.linalg as lin
    from scipy.spatial import distance
    
    #from matplotlib import pyplot
    
    
    #p =  'C:\Users\adity\Documents\NCSU\Proj1\testbox.jpeg'#'Desktop\\testbox.jpeg'
    img = cv2.imread(Image)
    S = img.shape
    #print('s', S)
    
    
    
    
    
    
    P1= np.array(P1)
    P2= np.array(P2)
    P3= np.array(P3)
    P4= np.array(P4)
    P5= np.array(P5)
    P6= np.array(P6)
    P7 = np.array(P7)
    
    
    w=1
    
    
    #Line 1 P1 & P2
    temp = [P1[0],P1[1],w]
    e1_1 = np.array(temp)
    temp = [P2[0],P2[1],w]
    e2_1 = np.array(temp)
    temp  = np.cross(e1_1, e2_1)
    l1 = np.array(temp)
    
    
    #Line 2 P1 & P2
    temp = [P3[0],P3[1],w]
    e1_2 = np.array(temp )
    temp = [P4[0],P4[1],w]
    e2_2 = np.array(temp)
    
    temp  = np.cross(e1_2, e2_2)
    l2 = np.array(temp)
    
    V1 =  np.array(np.cross(l1,l2))
    
    
    
    
    #Line 3 P1 & P3
    temp = [P1[0],P1[1],w]
    e1_3 = np.array(temp )
    temp = [P3[0],P3[1],w]
    e2_3 = np.array(temp)
    
    temp  = np.cross(e1_3, e2_3)
    l3 = np.array(temp)
    
    
    #Line 4 P2 & P4
    temp = [P2[0],P2[1],w]
    e1_4 = np.array(temp )
    temp = [P4[0],P4[1],w]
    e2_4 = np.array(temp)
    
    
    temp  = np.cross(e1_4, e2_4)
    l4 = np.array(temp)

    V2 =  np.array(np.cross(l3,l4))
    
    
     #Line 5 P3 & P5
    temp = [P3[0],P3[1],w]
    e1_5 = np.array(temp )
    temp = [P5[0],P5[1],w]
    e2_5 = np.array(temp)
    
    temp  = np.cross(e1_5, e2_5)
    l5 = np.array(temp)
    
    #Line 6 P4 & P6
    temp = [P4[0],P4[1],w]
    e1_6 = np.array(temp )
    temp = [P6[0],P6[1],w]
    e2_6 = np.array(temp)
    
    
    
    
    temp  = np.cross(e1_6, e2_6)
    l6 = np.array(temp)
    V3 = np.array(np.cross(l5,l6))
    
    
   
   
   
    
    tV1 = V1[:]/V1[2]

    tV2 = V2[:]/V2[2]
    tV3 = V3[:]/V3[2]
    
    #print('V1', V1)
    #print('V2', V2)
    #print('V3', V3)
    
    #Vanishing points
    Vy = np.array([tV2]).T
    Vx = np.array([tV3]).T
    Vz = np.array([tV1]).T
    
   # print('tV1', Vx)
   # print('tV2', Vy)
   # print('tV3', Vz)
    
    #World Origin in image-cords
    WO = np.array(W)
    WO = np.append([WO],[1])
    WO = np.array([WO]).T
    #Reference axis-cords in im-cords
    ref_x = np.array(X_ref)#[ 197 , 317 , 1 ]
    ref_x = np.append([ref_x],[1])
    x_ref =  np.array([ref_x]).T
    
    ref_y = np.array(Y_ref)#[ 442 , 317 , 1 ]
    ref_y = np.append([ref_y],[1])
    y_ref =  np.array([ref_y]).T 
    
    
    ref_z = np.array(Z_ref)#[ 319 , 227 , 1 ] #556  [ 778 , 400 , 1 ] 
    ref_z = np.append([ref_z],[1])
    z_ref =  np.array([ref_z]).T
    
    
    
    ref_x_dis = distance.euclidean(x_ref,WO)
    ref_y_dis = distance.euclidean(y_ref,WO)
    ref_z_dis = distance.euclidean(z_ref,WO)
    
    #print('Ref_distance_X',ref_x_dis)
    #print('Ref_distance_Y',ref_y_dis)
    #print('Ref_distance_Z',ref_z_dis)
    
    
    #%% Scaling factors of the projection matrix
    temp = np.array((x_ref - WO))
    tempx,resid,rank,s = np.linalg.lstsq((Vx-x_ref),temp)
    a_x = (tempx )  / ref_x_dis  #%( A \ B ==> left division )
    
    
    temp = np.array((y_ref - WO))
    tempy,resid,rank,s = np.linalg.lstsq((Vy-y_ref),temp)
    a_y = (tempy )  / ref_y_dis  #%( A \ B ==> left division )
    
    temp = np.array((z_ref - WO))
    tempz,resid,rank,s = np.linalg.lstsq((Vz-z_ref),temp)
    a_z = (tempz )  / ref_z_dis  #%( A \ B ==> left division )
    
    p1 = Vx*a_x
    p2 = Vy*a_y
    p3 = Vz*a_z
    p4 = np.array(WO)
    
    
    P = np.concatenate((p1, p2, p3, p4), axis =1)
    
    
    
    Hxy = np.concatenate((p1, p2, p4), axis =1)
    Hyz = np.concatenate((p2, p3, p4), axis =1)
    Hzx = np.concatenate((p1, p3, p4), axis =1)
    
    
    
    warp = cv2.warpPerspective(img, Hxy , (S[0]*5, S[1]*5), flags = cv2.WARP_INVERSE_MAP)
    warp1 = cv2.warpPerspective(img, Hyz , (S[0]*5, S[1]*5), flags = cv2.WARP_INVERSE_MAP)
    warp2 = cv2.warpPerspective(img, Hzx , (S[0], S[1]), flags = cv2.WARP_INVERSE_MAP)
    
    img_ann=cv2.line(img ,(P1[0],P1[1]),(P2[0],P2[1]), (0,255,0), 10)
    img_ann=cv2.line(img_ann ,(P3[0],P3[1]),(P4[0],P4[1]), (0,255,0), 10)
    img_ann=cv2.line(img_ann ,(P5[0],P5[1]),(P6[0],P6[1]), (0,255,0), 10)
    
    img_ann=cv2.line(img_ann ,(P1[0],P1[1]),(P3[0],P3[1]), (255,0,0), 10)
    img_ann=cv2.line(img_ann ,(P2[0],P2[1]),(P4[0],P4[1]), (255,0,0), 10)
    img_ann=cv2.line(img_ann ,(P7[0],P7[1]),(P5[0],P5[1]), (255,0,0), 10)
    
    img_ann=cv2.line(img_ann ,(P6[0],P6[1]),(P4[0],P4[1]), (0,0,255), 10)
    img_ann=cv2.line(img_ann ,(P3[0],P3[1]),(P5[0],P5[1]), (0,0,255), 10)
    img_ann=cv2.line(img_ann ,(P1[0],P1[1]),(P7[0],P7[1]), (0,0,255), 10)
    
    
    cv2.imshow('Ann', img_ann)
    cv2.imwrite('Ann.jpg', img_ann)
    cv2.imshow('XY', warp)
    cv2.imwrite('XY.jpg', warp)
    cv2.imshow('YZ', warp1)
    cv2.imwrite('YZ.jpg', warp1)
    cv2.imshow('ZX', warp2)
    cv2.imwrite('ZX.jpg', warp2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return ref_x_dis, ref_y_dis, ref_z_dis





#==============================================================================
def AutomaticAnnotation(Image):


    
    
    img1 = cv2.imread(Image,0)
    
    img1 =np.array(img1)
    
    
    
    
    #Create default parametrization LSD
    lsd = cv2.createLineSegmentDetector(0)
    
    #Detect lines in the image
    lines = lsd.detect(img1)[0] #Position 0 of the returned tuple are the detected lines
    
    #lines = np.array(lines)
    #Draw detected lines in the image
    drawn_img1 = lsd.drawSegments(img1,lines)
    
    
    
    l = len(lines)
    shape = lines.shape
    p1 = [0,0]#np.zeros
    p2 = [0,0]#np.zeros
    
    
    d1 = np.zeros(l)
    #LN = np.zeros((l,1,4))
    #LN = np.array(LN)
    LN = np.zeros_like(lines)
    #LN = np.zeros(shape)
    
    L = np.array(lines)
    threshold = 80
    
    
    p1 = np.array(p1)
    p2 = np.array(p2)
    d1 = np.array(d1)
    
    i=0
    k=0
    for i in range (0,(l-1)):
        
        for j in range(0,4):
            if j == 0:
                p1[0] = L[i,0,j]
            elif j == 1:
                p1[1] = L[i,0,j]
            elif j == 2:
                p2[0] = L[i,0,j]
            elif j == 3:
                p2[1] = L[i,0,j]
                
                
        d1[i]  = distance.euclidean(p1,p2)
        
        if d1[i] > threshold:
            #LN[k,0] = L[i,0]
            k=k+1
            
    LN = np.resize(LN,(k,1,4))    
    
    #LN = np.empty((k,1,4))
    #LN = np.zeros()
    
    #LN = np.array(LN)
    #LN = np.asanyarray(LN)
    
    
    k=0
    for i in range (0,(l-1)):
        
        for j in range(0,4):
            if j == 0:
                p1[0] = L[i,0,j]
            elif j == 1:
                p1[1] = L[i,0,j]
            elif j == 2:
                p2[0] = L[i,0,j]
            elif j == 3:
                p2[1] = L[i,0,j]
                
                
        d1[i]  = distance.euclidean(p1,p2)
        
        if d1[i] > threshold:
            
            LN[k,0] = lines[i,0]
            k=k+1
    
    '==========================Angle thresholding=================================='
    
    LN = np.array(LN)
    
    lines_new = LN
    LN_x = np.zeros_like(LN)
    LN_y = np.zeros_like(LN)
    LN_z = np.zeros_like(LN)     
            
    a = []
    x1_a = []
    x2_a=[]
    y1_a=[]
    y2_a=[]
    i=0
    q=0
    n_x = 0
    n_y = 0
    n_z = 0
    
    l = len(LN)
    for i in range (0,(l)):
        #q=q+1
        #k=k+1
        x1 = LN[i,0,0]
        x1_a.append(x1)
        
        y1 = LN[i,0,1]
        y1_a.append(y1)
        
        x2 = LN[i,0,2]
        x2_a.append(x2)
        
        y2 = LN[i,0,3]
        y2_a.append(y2)
        
        
        
        angle = math.atan2((y2-y1),(x2-x1))
        
        angle = math.degrees(angle)
        
        angle =  angle%360
        
        if angle > 180:
            angle = angle -180
        
      
        a.append(angle)
        
        
        if (angle > 75) & (angle < 105) :
            
            LN_z[n_z,0] = LN[i,0]
            n_z = n_z + 1
            
        if (angle > 25) & (angle < 50):
            
            LN_y[n_y,0] = LN[i,0]
            n_y = n_y + 1
            
        if (angle > 150) & (angle < 165):
            
            LN_x[n_x,0] = LN[i,0]
            n_x = n_x + 1
            
            
    
     
            
    
    
    
    #LN_x =  
    #LN = np.resize(LN,(k,1,4))
    
    
    LN_x = np.resize(LN_x,(n_x,1,4))
    LN_y = np.resize(LN_y,(n_y,1,4))        
    LN_z = np.resize(LN_z,(n_z,1,4)) 
    print(LN_x[0][0][0])
    
    #LN_Z = np.array(LN_z)
    
    
     
    [n1_x,n2_x,n3_x]=np.shape(LN_x)
    k_means_x=[[]for i in range(2*n1_x)]
    p=0
    for i in range(n1_x):
        k_means_x[p].append(LN_x[i][0][0])
        k_means_x[p].append(LN_x[i][0][1])
        p=p+1
        k_means_x[p].append(LN_x[i][0][2])
        k_means_x[p].append(LN_x[i][0][3])
        p=p+1
    
    
    [n1_y,n2_y,n3_y]=np.shape(LN_y)
    k_means_y=[[]for i in range(2*n1_y)]
    p=0
    for i in range(n1_y):
        k_means_y[p].append(LN_y[i][0][0])
        k_means_y[p].append(LN_y[i][0][1])
        p=p+1
        k_means_y[p].append(LN_y[i][0][2])
        k_means_y[p].append(LN_y[i][0][3])
        p=p+1
    
    
    
    [n1_z,n2_z,n3_z]=np.shape(LN_z)
    k_means_z=[[]for i in range(2*n1_z)]
    p=0
    for i in range(n1_z):
        k_means_z[p].append(LN_z[i][0][0])
        k_means_z[p].append(LN_z[i][0][1])
        p=p+1
        k_means_z[p].append(LN_z[i][0][2])
        k_means_z[p].append(LN_z[i][0][3])
        p=p+1
    kmeans = KMeans(n_clusters=3, random_state=0).fit(k_means_z)
    labels_z=kmeans.labels_
    
    kmeans = KMeans(n_clusters=3, random_state=0).fit(k_means_y)
    labels_y=kmeans.labels_
        
    
    modified_img1 = lsd.drawSegments(img1,LN)
    
    #Show image
    drawn_img1 = cv2.resize(drawn_img1, (1000,1000),1,1, cv2.INTER_AREA)
    
    modified_img1 = cv2.resize(modified_img1, (1000,1000),1,1, cv2.INTER_AREA)
    
    modified_img1_x = lsd.drawSegments(img1,LN_x) 
    modified_img1_y = lsd.drawSegments(img1,LN_y) 
    modified_img1_z = lsd.drawSegments(img1,LN_z) 
      
       
    
    
    
    
    cv2.imshow("LSD",drawn_img1)
    cv2.imwrite("LSD.jpg",drawn_img1)
    cv2.imshow("Edges",modified_img1)
    #cv2.imshow("Mod_LSD_X",modified_img1_x)
    #cv2.imshow("Mod_LSD_Y",modified_img1_y)
    #cv2.imshow("Mod_LSD_Z",modified_img1_z)
    
    
    
    Zref_X_x = LN_x[1,0,0]
    Zref_X_y = LN_x[1,0,1]
    
    
    Zref_Y_x = LN_y[1,0,2]
    Zref_Y_y = LN_y[1,0,3]
    
    
    Zref_Z_x = LN_z[2,0,0]
    Zref_Z_y = LN_z[2,0,1]
    
    Zrefx = np.array([Zref_X_x,Zref_Y_x,Zref_Z_x])
    Zrefy = np.array([Zref_X_y,Zref_Y_y,Zref_Z_y])
    
    
    Zrefx = np.average(Zrefx)
    
    Zrefy = np.average(Zrefy)
    
    
    
    
    Xref_X_x = LN_x[2,0,2]
    Xref_X_y = LN_x[2,0,3]
    
    
    #Xref_Y_x = LN_y[1,0,2]
    #Xref_Y_y = LN_y[1,0,3]
    
    
    Xref_Z_x = LN_z[1,0,0]
    Xref_Z_y = LN_z[1,0,1]
    
    Xrefx = np.array([Xref_X_x,Xref_Z_x])
    Xrefy = np.array([Xref_X_y,Xref_Z_y])
    
    
    Xrefx = np.average(Xrefx)
    
    Xrefy = np.average(Xrefy)
    
    
    
    
    
    #Yref_X_x = LN_x[0,0,2]
    #Yref_X_y = LN_x[0,0,3]
    
    
    Yref_Y_x = LN_y[2,0,0]
    Yref_Y_y = LN_y[2,0,1]
    
    
    Yref_Z_x = LN_z[0,0,2]
    Yref_Z_y = LN_z[0,0,3]
    
    Yrefx = np.array([Yref_Y_x,Yref_Y_x])
    Yrefy = np.array([Yref_Y_y,Yref_Y_y])
    
    
    Yrefx = np.average(Yrefx)
    
    Yrefy = np.average(Yrefy)
    
    
    
    
    
    
    W_X_x = LN_x[2,0,0]
    W_X_y = LN_x[2,0,1]
    
    
    W_Y_x = LN_y[2,0,2]
    W_Y_y = LN_y[2,0,3]
    
    
    W_Z_x = LN_z[2,0,2]
    W_Z_y = LN_z[2,0,3]
    
    Wx = np.array([W_X_x,W_Y_x,W_Z_x])
    Wy = np.array([W_X_y,W_Y_y,W_Z_y])
    
    
    Wx = np.average(Wx)
    
    Wy = np.average(Wy)
    
    
    
    P1_X_x = LN_x[0,0,2]
    P1_X_y = LN_x[0,0,3]
    
    
    P1_Y_x = LN_y[1,0,0]
    P1_Y_y = LN_y[1,0,1]
    
    
    P1_Z_x = LN_z[0,0,0]
    P1_Z_y = LN_z[0,0,1]
    
    P1x = np.array([P1_X_x,P1_Y_x,P1_Z_x])
    P1y = np.array([P1_X_y,P1_Y_y,P1_Z_y])
    
    
    P1x = np.average(P1x)
    
    P1y = np.average(P1y)
    
    
    
    P5_X_x = LN_x[1,0,2]
    P5_X_y = LN_x[1,0,3]
    
    
    P5_Y_x = LN_y[0,0,0]
    P5_Y_y = LN_y[0,0,1]
    
    
    P5_Z_x = LN_z[1,0,2]
    P5_Z_y = LN_z[1,0,3]
    
    P5x = np.array([P5_X_x,P5_Y_x,P5_Z_x])
    P5y = np.array([P5_X_y,P5_Y_y,P5_Z_y])
    
    
    P5x = np.average(P5x)
    
    P5y = np.average(P5y)
    
    
    
    
    P7_X_x = LN_x[0,0,0]
    P7_X_y = LN_x[0,0,1]
    
    
    P7_Y_x = LN_y[0,0,2]
    P7_Y_y = LN_y[0,0,3]
    
    
    #P7_Z_x = LN_z[0,0,0]
    #P7_Z_y = LN_z[0,0,1]
    
    P7x = np.array([P7_X_x,P7_Y_x])
    P7y = np.array([P7_X_y,P7_Y_y])
    
    
    P7x = np.average(P7x)
    
    P7y = np.average(P7y)
    
    
    
    
    X_ref = [Xrefx, Xrefy]
    Y_ref = [Yrefx, Yrefy]
    Z_ref = [Zrefx, Zrefy]
    P1 = [P1x,P1y]
    P5 = [P5x,P5y]
    P7 = [P7x,P7y]
    W = [Wx,Wy]
    
    
    
    
    #plt.imshow(drawn_img)
    #plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
    #plt.show()
    '''
    # Robustly fit linear model with RANSAC algorithm
    ransac = linear_model.RANSACRegressor()
    ransac.fit(X, y)
    inlier_mask = ransac.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)
    
    '''
    
    #P1 
    P2 = Y_ref
    P3 = Z_ref
    P4 = W
    #P5 =
    P6 = X_ref
    #P7 =
    
    
    
    
    ref_dis_x, ref_dis_y, ref_dis_z = SingleViewGeom(Image, P1, P2, P3, P4, P5, P6, P7, X_ref, Y_ref, Z_ref, W)
    
    
    
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return



#path = 'C:\\Users\\adity\\Documents\\NCSU\\1ECE558\\Proj2\\arasam_project01(1)\\arasam_project01\\arasam_code\\testbox.jpg'
#Image = cv2.imread(path)
#Image = path



#AutomaticAnnotation('testbox.jpg')


