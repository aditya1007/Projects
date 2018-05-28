function [mymsk] = my_background_mask(inputImg)

% % >>>> for debug
% folderName = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\TrainingSetRefinedMat\';
% fileName = '001_sample.mat';
% load([folderName fileName]);
% inputImg = prob_map;

binImg = imbinarize(inputImg, 20);

cumulMat(:,:, 1) = cumsum(inputImg, 2);
cumulMat(:,:, 2) = cumsum(inputImg, 2, 'reverse');
cumulMat(:,:, 3) = cumsum(inputImg, 1);
cumulMat(:,:, 4) = cumsum(inputImg, 1, 'reverse');

mymsk = min(cumulMat, [], 3);
