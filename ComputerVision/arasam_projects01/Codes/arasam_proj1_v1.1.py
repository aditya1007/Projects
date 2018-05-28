import cv2
import os.path
import numpy as np
import sqlite3



data_retrieve_path = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/flickr/"
data_store_path = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/"



index = 1


sqldata = sqlite3.connect('C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/aflw.sqlite')

s = sqldata.cursor()

#Creating the query string for retriving: roll, pitch, yaw and faces position
#Change it according to what you want to retrieve
column_string = "faceimages.filepath, faces.face_id, facepose.roll, facepose.pitch, facepose.yaw, facerect.x, facerect.y, facerect.w, facerect.h"
table_string = "faceimages, faces, facepose, facerect"
row_string = "faces.face_id = facepose.face_id and faces.file_id = faceimages.file_id and faces.face_id = facerect.face_id"
query_string = "SELECT " + column_string + " FROM " + table_string + " WHERE " + row_string

for row in s.execute(query_string):

    input_data_path = data_retrieve_path + str(row[0])
    output_data_path = data_store_path + '3/'+'img_' + str(index) + '.jpg'
    output_data_path_2 = "C:/Users/adity/Documents/NCSU/0 Semester 4/1CV/Project1/Data/aflw/data/output/nonface" + '3/'+'img_' + str(index) + '.jpg'

    #Check for existence of file       
    if(os.path.isfile(input_data_path)  == True):
        image = cv2.imread(input_data_path, 1) 
        

        #Image dimensions
        img_height, img_width, img_channel = image.shape
        
        #Face rectangle coords
        face_x_ord = row[5]
        face_y_ord = row[6]
        face_width = row[7]
        face_height = row[8]


        #Error correction
        if(face_x_ord < 0): face_x_ord = 0
        if(face_y_ord < 0): face_y_ord = 0
        if(face_width > img_width): 
            face_width = img_width
            face_height = img_width
        if(face_height > img_height): 
            face_height = img_height
            face_width = img_height

        #Crop the face from the image
        image_face_cropped = np.copy(image[face_y_ord:face_y_ord+face_height, face_x_ord:face_x_ord+face_width])
        
        img_face = np.array(image_face_cropped)
        
        vector_1 = img_face.flatten()
            
            
        
        image_nonface_cropped = np.copy(image[0:60,0:60])
        
        #Rescaling the image
        size = 60
        image_face_rescaled = cv2.resize(image_face_cropped, (size,size), interpolation = cv2.INTER_AREA)
        #image_nonface_rescaled = cv2.resize(image_nonface_cropped, (size,size), interpolation = cv2.INTER_AREA)
        
        cv2.imwrite(output_data_path, image_face_rescaled)
        cv2.imwrite(output_data_path_2, image_nonface_cropped)

        print ("Image Number: " + str(index))
        print ('Path', output_data_path)
        print ('Path2', output_data_path_2)
        
        
        
        index =  index + 1
        #if the file does not exits it return an exception
    else:
        raise ValueError('Error: Unable to access the file: ' + str(input_data_path))
        
    if (index >= 1200):
            break

s.close()
print('New')



    



    
    
