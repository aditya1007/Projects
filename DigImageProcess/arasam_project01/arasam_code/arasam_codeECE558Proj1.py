# -*- coding: utf-8 -*-
"""
Created on Sat Oct 21 22:05:00 2017

@author: aditya
"""



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
    
    
    
    warp = cv2.warpPerspective(img, Hxy , (S[0], S[1]), flags = cv2.WARP_INVERSE_MAP)
    warp1 = cv2.warpPerspective(img, Hyz , (S[0], S[1]), flags = cv2.WARP_INVERSE_MAP)
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
    cv2.imshow('XY', warp)
    cv2.imshow('YZ', warp1)
    cv2.imshow('ZX', warp2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return

    
    
