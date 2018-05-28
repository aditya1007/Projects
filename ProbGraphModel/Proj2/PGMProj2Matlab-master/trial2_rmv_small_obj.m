I = imread('001_sample_DOG1.jpg');
I2 = bwareaopen(I>150, 100);
figure(1);surf(I, 'edgecolor', 'none')
figure(2);surf(double(I2), 'edgecolor', 'none')