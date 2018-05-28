#!/usr/bin/env python

##
# Massimiliano Patacchiola, Plymouth University 2016
# website: http://mpatacchiola.github.io/
# email: massimiliano.patacchiola@plymouth.ac.uk
# Python code for information retrieval from the Annotated Facial Landmarks in the Wild (AFLW) dataset.
# In this example the faces are isolated and saved in a specified output folder.
# Some information (roll, pitch, yaw) are returned, they can be used to filter the images.
# This code requires OpenCV and Numpy. You can easily bypass the OpenCV calls if you want to use
# a different library. In order to use the code you have to unzip the images and store them in
# the directory "flickr" mantaining the original folders name (0, 2, 3).
#
# The following are the database properties available (last updated version 2012-11-28):
#
# databases: db_id, path, description
# faceellipse: face_id, x, y, ra, rb, theta, annot_type_id, upsidedown
# faceimages: image_id, db_id, file_id, filepath, bw, widht, height
# facemetadata: face_id, sex, occluded, glasses, bw, annot_type_id
# facepose: face_id, roll, pitch, yaw, annot_type_id
# facerect: face_id, x, y, w, h, annot_type_id
# faces: face_id, file_id, db_id
# featurecoords: face_id, feature_id, x, y
# featurecoordtype: feature_id, descr, code, x, y, z

import cv2
import os.path
import numpy as np
import sqlite3
import csv

data_retrieve_path = "/home/aditya/ADITYA/CV/Proj2/Data/aflw/data/flickr/"
data_store_path_f = "/home/aditya/ADITYA/CV/Proj2/Data/aflw/data/output/face/"
data_store_path_nf = "/home/aditya/ADITYA/CV/Proj2/Data/aflw/data/output/nonface/"


def main(NoOfTrainData, NoOfTestData):
    index = 1

    sqldata = sqlite3.connect('/home/aditya/ADITYA/CV/Proj2/Data/aflw/data/aflw.sqlite')

    s = sqldata.cursor()

    # Creating the query string for retriving: roll, pitch, yaw and faces position
    # Change it according to what you want to retrieve
    column_string = "faceimages.filepath, faces.face_id, facepose.roll, facepose.pitch, facepose.yaw, facerect.x, facerect.y, facerect.w, facerect.h"
    table_string = "faceimages, faces, facepose, facerect"
    row_string = "faces.face_id = facepose.face_id and faces.file_id = faceimages.file_id and faces.face_id = facerect.face_id"
    query_string = "SELECT " + column_string + " FROM " + table_string + " WHERE " + row_string

    i = 0
    for row in s.execute(query_string):

        input_data_path = data_retrieve_path + str(row[0])


        # Check for existence of file
        if (os.path.isfile(input_data_path) == True):
            image = cv2.imread(input_data_path, 1)

            # Image dimensions
            img_height, img_width, img_channel = image.shape

            # Face rectangle coords
            face_x_ord = row[5]
            face_y_ord = row[6]
            face_width = row[7]
            face_height = row[8]

            # Error correction
            if (face_x_ord < 0): face_x_ord = 0
            if (face_y_ord < 0): face_y_ord = 0
            if (face_width > img_width):
                face_width = img_width
                face_height = img_width
            if (face_height > img_height):
                face_height = img_height
                face_width = img_height

            # Crop the face from the image
            image_face_cropped = np.copy(image[face_y_ord:face_y_ord + face_height, face_x_ord:face_x_ord + face_width])

            ''' 
            if ((face_x_ord+2*face_width) < img_width):

                nonface_start_x = face_x_ord+face_width
                nonface_end_x = face_x_ord+2*face_width

                nonface_start_y = face_y_ord
                nonface_end_y = face_y_ord+face_height

                if ((face_y_ord+2*face_height) < img_height):
                    nonface_start_y = face_y_ord+face_height
                    nonface_end_y = face_y_ord+2*face_height

            elif ((face_y_ord+2*face_height) < img_height):
                nonface_start_y = face_y_ord+face_height
                nonface_end_y = face_y_ord+2*face_height

                nonface_start_x = face_x_ord
                nonface_end_x = face_x_ord+face_width


                if ((face_x_ord+2*face_width) < img_width):

                    nonface_start_x = face_x_ord+face_width
                    nonface_end_x = face_x_ord+2*face_width






            if ((face_x_ord+2*face_width) < img_width)&((face_y_ord+2*face_height) < img_height):

                nonface_start_x = face_x_ord+face_width
                nonface_end_x = face_x_ord+2*face_width

                nonface_start_y = face_y_ord+face_height
                nonface_end_y = face_y_ord+2*face_height

            image_nonface_cropped = np.copy(image[nonface_start_y:nonface_end_y, nonface_start_x:nonface_end_x])

            '''

            image_nonface_cropped = np.copy(image[0:60, 0:60])
            image_nonface_cropped = cv2.cvtColor(image_nonface_cropped, cv2.COLOR_BGR2GRAY)

            # Rescaling the image
            size = 60
            image_face_rescaled = cv2.resize(image_face_cropped, (size, size), interpolation=cv2.INTER_AREA)
            # image_nonface_rescaled = cv2.resize(image_nonface_cropped, (size,size), interpolation = cv2.INTER_AREA)

            image_face_rescaled = cv2.cvtColor(image_face_rescaled, cv2.COLOR_BGR2GRAY)

            if index < (NoOfTrainData + 1):
                output_data_path = data_store_path_f + 'train/' + str(index) + '.jpg'  # str(row[0])
                output_data_path_2 = data_store_path_nf + 'train/' + str(index) + '.jpg'  # str(row[0])  # data_store_path +'nonface'


                cv2.imwrite(output_data_path, image_face_rescaled)
                cv2.imwrite(output_data_path_2, image_nonface_cropped)



                print ("Image Number: " + str(index))
                print(row[0])
                print ('Path', output_data_path)
                print ('Path2', output_data_path_2)

            elif (index >= NoOfTrainData + 1) & (index < NoOfTrainData + NoOfTestData +1):
                output_data_path = data_store_path_f + 'test/' + str(index) + '.jpg'  # str(row[0])
                output_data_path_2 = data_store_path_nf + 'test/' + str(index) + '.jpg'  # str(row[0])  # data_store_path +'nonface'

                cv2.imwrite(output_data_path, image_face_rescaled)
                cv2.imwrite(output_data_path_2, image_nonface_cropped)


                print ("Image Number: " + str(index))
                print(row[0])
                print ('Path', output_data_path)
                print ('Path2', output_data_path_2)


            i = i + 1

            index = index + 1
            # if the file does not exits it return an exception
        else:
            raise ValueError('Error: Unable to access the file: ' + str(input_data_path))

        if (index >= NoOfTrainData + NoOfTestData +1):
            break

    s.close()
    print('New')

def GenCSV(dataType):

    ImageType  = 'face'
    temp_data_store_path_face = "/home/aditya/ADITYA/CV/Proj2/Data/aflw/data/output/" + ImageType + "/" + dataType + "/"

    data_train_array = []
    label_train_array = []

    for name in os.listdir(temp_data_store_path_face):
        c = name
        a = c.split(".")
        #print (c)
        #print(a[0])

        data_train_array.append(temp_data_store_path_face + c)
        if ImageType == 'face':
            label_train_array.append(1)
        elif ImageType == 'nonface':
            label_train_array.append(0)


    filename = dataType + '_dataset.csv'

    myFile = open(filename, 'w')
    with myFile:
        writer = csv.writer(myFile)
        writer.writerows(zip(data_train_array,label_train_array))

    ImageType = 'nonface'
    temp_data_store_path_face = "/home/aditya/ADITYA/CV/Proj2/Data/aflw/data/output/" + ImageType + "/" + dataType + "/"
    for name in os.listdir(temp_data_store_path_face):
        c = name
        a = c.split(".")
            # print (c)
            # print(a[0])

        data_train_array.append(temp_data_store_path_face + c)
        if ImageType == 'face':
            label_train_array.append(1)
        elif ImageType == 'nonface':
            label_train_array.append(0)

    filename = dataType + '_dataset.csv'

    myFile = open(filename, 'w')
    with myFile:
        writer = csv.writer(myFile)
        writer.writerows(zip(data_train_array, label_train_array))




        #print(name)



if __name__ == "__main__":
    #main(10000,1000)                         #UnCOMMENT to generate 60x60 images data set
    print(1)
    GenCSV('train')
    print(2)
    GenCSV('test')
    #print(3)
    #GenCSV('nonface', 'train')
    #print(4)
    #GenCSV('nonface', 'test')

