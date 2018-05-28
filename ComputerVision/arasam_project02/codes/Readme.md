To run the code 

1. Generate the csv file using Data_Set_Gen.py
   Provide the path for the image data retrieval in 'data_retrieve_path ', path for storing the images for face and non-face in 'data_store_path_f' and 'data_store_path_nf' variable.Path for csv file storage is to be given at 'temp_data_store_path_face '. 

2. Run the code python file'CNN_Face_Detection_V2.py'
   The path for the Data set is mensioned in the csv files. Update ths path in train dataset loader
'train_dataset = CustomDatasetFromImages('/home/aditya/PycharmProjects/ECE763_Proj2/train_dataset.csv')'

3. For running the random serach run the python file 'CNN_Face_Detection_Random_Search_V3'
