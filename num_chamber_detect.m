function [numChambers, chamberCenters, logMedImg, wtrShdMskImg, bkgroundMsk, binImage, binImage1] = num_chamber_detect(inputImg)

% >>>> for debug
% folderName = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\TrainingSetRefinedMat\';
% fileName = '001_sample.mat';
% load([folderName fileName]);
% inputImg = prob_map;

% thresholding
%T = adaptthresh(inputImg, 0.01);
T = adaptthresh(inputImg, 0.5);
%minT = 0.6;
%minT = 0.3;
minT = 0.4;
T(T<minT) = minT;
binImage = imbinarize(inputImg, T);

% distance transform
binImage1 = bwareaopen(binImage, 10);

% distance transform
distTranImage = bwdist(binImage1, 'euclidean');

% optional gaussian filter [TODO]

%figure;imagesc(distTranImage);

% apply background mask
%[bkgroundMsk] = background_masking(inputImg);
[bkgroundMsk] = my_background_mask(inputImg);

% shrink background mask
%bkgroundMsk = imerode(bkgroundMsk, strel('disk', 2));

mskdImg = min(bkgroundMsk*255, uint8(distTranImage));

%  log transform
[logImg] = my_log_transform(mskdImg);

logMedImg = medfilt2(logImg, [7 7]);
% Try bilateral filter [TODO]

% Apply DoG [TODO]

% >>>> for debug
%logMedImg = 1-double(inputImg)/max(double(inputImg(:)));

wtrShdImg = watershed(1-logMedImg);
wtrShdMskImg = min(bkgroundMsk, wtrShdImg);

%logImgOpen = bwareaopen(logImg, 10);
%figure;
%surf(logMedImg, 'edgecolor', 'none');

% counting the number of disconnected objects
%cc = bwconncomp(flipud(logMedImg));
cc = bwconncomp(wtrShdMskImg);

numChambers = cc.NumObjects;
centroidStruct = regionprops(cc, 'centroid');
chamberCenters = [];
for ind = 1:numChambers
    chamberCenters = [chamberCenters;centroidStruct(ind).Centroid];
end

chamberCenters = round(chamberCenters);


% figure(100);
% subplot(221);imagesc(binImage);
% subplot(222);imagesc(binImage1);
% subplot(223);imagesc(logImg);
% subplot(224);imagesc(logMedImg);
% 
% brkpnt1 = 1;


