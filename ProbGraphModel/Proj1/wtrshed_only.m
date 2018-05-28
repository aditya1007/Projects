clear all;close all;clc;
%folderNameMat = 'TrainingSetRefinedMat/';
folderNameMat = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\TrainingSetRefinedMat\';
samplesList = 1:320;
%samplesList = 18;
imgSize = 230;
coveringMetricAll = zeros(1, length(samplesList));

% % load bad samples
% load('badSampList.mat');
% samplesList = badSampList;

for indSamp = 1:length(samplesList)
    %% Loading the file
    sampleNum  = samplesList(indSamp);
    fprintf('Sample number = %d\n', sampleNum);
    if sampleNum < 10
        fileName = ['00' num2str(sampleNum) '_sample.mat'];
        fileNamelabel = ['00' num2str(sampleNum) '_sample_label.mat'];
    elseif sampleNum < 100
        fileName = ['0' num2str(sampleNum) '_sample.mat'];
        fileNamelabel = ['0' num2str(sampleNum) '_sample_label.mat'];
    else
        fileName = [num2str(sampleNum) '_sample.mat'];
        fileNamelabel = [num2str(sampleNum) '_sample_label.mat'];
    end
    load([folderNameMat fileName]);
    load([folderNameMat fileNamelabel]);
    
    %figure(1);imagesc(label_im);
    brkpnt1 = 1;
    %continue;
    
    % downsample
%     imgSize = 70;
%     prob_map = imresize(prob_map, [imgSize,imgSize]);
    
    %% detecting the number of chambers and centroids
    [numChambers, chamberCenters, logMedImg, wtrShdMskImg, bkgroundMsk] = num_chamber_detect(prob_map);
    centroids = fliplr(chamberCenters);
    
    wtrShdMskModefilImg = colfilt(wtrShdMskImg, [10,10], 'sliding', @mode);
    wtrShdMskModefilImg = min(bkgroundMsk, wtrShdMskModefilImg); % Masking again to prevent extra growth (check performance by considering it)
    
    %coveringMetric = get_covering_metric
    [coveringMetric] = get_covering_metric(label_im, wtrShdMskModefilImg);
    coveringMetricAll(indSamp) = coveringMetric;
    
%     figure(1);
%     if sampleNum == 317
%         subplot(231);imagesc(prob_map);
%         subplot(232);imagesc(label_im);
%         subplot(233);imagesc(wtrShdMskModefilImg);
%     elseif sampleNum == 302
%         subplot(234);imagesc(prob_map);
%         subplot(235);imagesc(label_im);
%         subplot(236);imagesc(wtrShdMskModefilImg);
%     end
    
end


