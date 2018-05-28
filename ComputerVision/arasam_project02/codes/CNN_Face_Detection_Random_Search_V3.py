
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import torchvision.datasets as dsets
from torch.autograd import Variable
from torch.utils.data.dataset import Dataset
'''
LIBRARIES OTHER THAN TORCH
'''
from PIL import Image
import pandas as pd
import numpy as np
import sys
from random import uniform


'''
-------------------------------CLASS FOR CUSTOM DATA SET--------------------------------------------------------------------
'''


class CustomDatasetFromImages(Dataset):
    def __init__(self, csv_path):
        """
        Args:
            csv_path (string): path to csv file
            img_path (string): path to the folder where images are
            transform: pytorch transforms for transforms and tensor conversion
        """
        # Transforms
        self.to_tensor = transforms.ToTensor()
        # Read the csv file
        self.data_info = pd.read_csv(csv_path, header=None)
        # First column contains the image paths
        self.image_arr = np.asarray(self.data_info.iloc[:, 0])
        # print(self.image_arr)
        # Second column is the labels
        self.label_arr = np.asarray(self.data_info.iloc[:, 1])
        #print(self.label_arr)
        # Third column is for an operation indicator
        #self.operation_arr = np.asarray(self.data_info.iloc[:, 2])
        # Calculate len
        self.data_len = len(self.data_info.index)
        #################################################################
        #print(self.data_len)
        #print(sys.getsizeof(self.label_arr))

    def __getitem__(self, index):
        # Get image name from the pandas df
        single_image_name = self.image_arr[index]
        #print(single_image_name)
        # Open image
        img_as_img = Image.open(single_image_name)
        #print(img_as_img)

        # Check if there is an operation
        #some_operation = self.operation_arr[index]
        # If there is an operation
        #if some_operation:
            # Do some operation on image
            # ...
            # ...
        #    pass
        # Transform image to tensor
        img_as_tensor = self.to_tensor(img_as_img)
        #print(img_as_tensor)
        # Get label(class) of the image based on the cropped pandas column
        single_image_label = self.label_arr[index]
        #print(single_image_label)

        return (img_as_tensor, single_image_label)

    def __len__(self):
        return self.data_len


'''
class MyDataSet(Dataset):
    """

    Arguments:
        A CSV file path
        Path to image folder
        Extension of images
        PIL transforms
    """

    def __init__(self, csv_path, img_path, img_ext, transform=None):
        tmp_df = pd.read_csv(csv_path)
        assert tmp_df['image_name'].apply(lambda x: os.path.isfile(img_path + x + img_ext)).all(), \
            "Some images referenced in the CSV file were not found"

        self.mlb = MultiLabelBinarizer()
        self.img_path = img_path
        self.img_ext = img_ext
        self.transform = transform

        self.X_train = tmp_df['image_name']
        self.y_train = self.mlb.fit_transform(tmp_df['tags'].str.split()).astype(np.float32)

    def __getitem__(self, index):
        img = Image.open(self.img_path + self.X_train[index] + self.img_ext)
        img = img.convert('RGB')
        if self.transform is not None:
            img = self.transform(img)

        label = torch.from_numpy(self.y_train[index])
        return img, label

    def __len__(self):
        return len(self.X_train.index)
'''


'''
STEP 1: LOADING DATASET ________________________________________________________________________________________________
'''


#transformations = transforms.Compose([transforms.Scale(32),transforms.ToTensor()])

#transformations = transforms.Compose([transforms.ToTensor()])
train_dataset = CustomDatasetFromImages('/home/aditya/PycharmProjects/ECE763_Proj2/train_dataset.csv')

test_dataset = CustomDatasetFromImages('/home/aditya/PycharmProjects/ECE763_Proj2/test_dataset.csv')
#print(train_dataset.data_len)
#print(sys.getsizeof(train_dataset.label_arr))



'''
train_dataset = dsets.CustomDatasetFromCSV(root= '/home/aditya/ADITYA/CV/Proj2/Data/aflw/data/output/3', # './data',
                            #train=True,
                            transform=transforms.ToTensor())
                            #download=True)

test_dataset = dsets.CustomDatasetFromCSV(root='/home/aditya/ADITYA/CV/Proj2/Data/aflw/data/output/2',
                           #train=False,
                           transform=transforms.ToTensor())
'''


'''
STEP 2: MAKING DATASET ITERABLE ________________________________________________________________________________________
'''

batch_size = 100
n_iters = 3000
num_epochs = n_iters / (len(train_dataset) / batch_size)
num_epochs = int(num_epochs)
print(num_epochs)

train_loader = torch.utils.data.DataLoader(dataset=train_dataset,
                                           batch_size=batch_size,
                                           shuffle=True)

test_loader = torch.utils.data.DataLoader(dataset=test_dataset,
                                          batch_size=batch_size,
                                          shuffle=False)

'''
STEP 3: CREATE MODEL CLASS______________________________________________________________________________________________
'''


class CNNModel(nn.Module):
    def __init__(self):
        super(CNNModel, self).__init__()

        # Convolution 1
        self.cnn1 = nn.Conv2d(in_channels=1, out_channels=16, kernel_size=5, stride=1, padding=0)
        self.relu1 = nn.ReLU()

        # Max pool 1
        self.maxpool1 = nn.MaxPool2d(kernel_size=2)

        # Convolution 2
        self.cnn2 = nn.Conv2d(in_channels=16, out_channels=32, kernel_size=5, stride=1, padding=0)   #32
        self.relu2 = nn.ReLU()

        # Max pool 2
        self.maxpool2 = nn.MaxPool2d(kernel_size=2)

        # Fully connected 1 (readout)
        self.fc1 = nn.Linear(32 * 12 * 12, 2)   #10

    def forward(self, x):
        #print(x)
        # Convolution 1
        out = self.cnn1(x)
        out = self.relu1(out)

        # Max pool 1
        out = self.maxpool1(out)

        # Convolution 2
        out = self.cnn2(out)
        out = self.relu2(out)

        # Max pool 2
        out = self.maxpool2(out)
        #print(out.size())

        # Resize
        # Original size: (100, 32, 7, 7)
        # out.size(0): 100
        # New out size: (100, 32*7*7)
        out = out.view(out.size(0), -1)

        # Linear function (readout)
        out = self.fc1(out)

        return out



'''
STEP 4: INSTANTIATE MODEL CLASS_____________________________________________________________________________________
'''

#model = CNNModel()





'''
#######################
#  USE GPU FOR MODEL  #
#######################

if torch.cuda.is_available():
    model.cuda()
'''




'''
STEP 5: INSTANTIATE LOSS CLASS
'''
#criterion = nn.CrossEntropyLoss()




'''
STEP 6: INSTANTIATE OPTIMIZER CLASS
'''
#learning_rate = 0.001
#reg = 0.001

#optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate, weight_decay=reg)

for i in range(50):
    model = CNNModel()
    print(i)

    #######################
    #  USE GPU FOR MODEL  #
    #######################
    criterion = nn.CrossEntropyLoss()

    l = 10 ** uniform(-3, -4)
    r = 10 ** uniform(-4, 0)

    learning_rate = l


    optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate, weight_decay=r)
    #optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate, weight_decay=r)
    # optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    '''
    STEP 7: TRAIN THE MODEL
    '''


    iter = 0
    for epoch in range(num_epochs):
        print('Epoch: ',epoch)
        for i, (images, labels) in enumerate(train_loader):

            #######################
            #  USE GPU FOR MODEL  #
            #######################
            #if torch.cuda.is_available():
            #    images = Variable(images.cuda())
            #    labels = Variable(labels.cuda())
            #else:
            images = Variable(images)
            labels = Variable(labels)

            # Clear gradients w.r.t. parameters
            optimizer.zero_grad()

            # Forward pass to get output/logits
            outputs = model(images)
            #print(i)

            # Calculate Loss: softmax --> cross entropy loss
            loss = criterion(outputs, labels)

            # Getting gradients w.r.t. parameters
            loss.backward()

            # Updating parameters
            optimizer.step()

            iter += 1

            if iter % 500 == 0:
                # Calculate Accuracy
                correct = 0
                total = 0

                # Iterate through test dataset
                for images, labels in test_loader:
                    #######################
                    #  USE GPU FOR MODEL  #
                    #######################
                    #if torch.cuda.is_available():
                    #    images = Variable(images.cuda())
                    #else:
                    images = Variable(images)

                    # Forward pass only to get logits/output
                    outputs = model(images)

                    # Get predictions from the maximum value
                    _, predicted = torch.max(outputs.data, 1)

                    # Total number of labels
                    total += labels.size(0)

                    #######################
                    #  USE GPU FOR MODEL  #
                    #######################
                    # Total correct predictions
                    #if torch.cuda.is_available():
                    #    correct += (predicted.cpu() == labels.cpu()).sum()
                    #else:
                    correct += (predicted == labels).sum()

                accuracy = 100 * correct / total

                # Print Loss
                #print('Iteration: {}. Loss: {}. Accuracy: {}'.format(iter, loss.data[0], accuracy))
                print('Iteration: {}. Loss: {}. Accuracy: {} learning rate:{} reg :{}'.format(iter, loss.data[0], accuracy, l, r))
