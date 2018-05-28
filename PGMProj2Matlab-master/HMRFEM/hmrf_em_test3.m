clear;clc;close all;

mex BoundMirrorExpand.cpp;
mex BoundMirrorShrink.cpp;

folderNameMat = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\TrainingSetRefinedMat\';
folderNameAdi = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\Processed_Data_Set_11_28_0100hrs\';

%load('badSampList.mat');samplesList = badSampList;
samplesList = 1:320;

coveringMetricHMRFAll = zeros(1, length(samplesList));
coveringMetricWtrShdAll = zeros(1, length(samplesList));

EM_iter=5; % max num of iterations
MAP_iter=5; % max num of iterations
numCircshift = 3;kDeltaInit = 3;

for indSamp = 1:length(samplesList)
    
    sampleNum  = samplesList(indSamp);
    if sampleNum < 10
        fileName = ['00' num2str(sampleNum) '_sample'];
    elseif sampleNum < 100
        fileName = ['0' num2str(sampleNum) '_sample'];
    else
        fileName = [num2str(sampleNum) '_sample'];
    end
    
    fileNameLabel = [fileName '_label'];
    load([folderNameMat fileName '.mat']);
    load([folderNameMat fileNameLabel '.mat']);
    fileNameAdi = [fileName '_skel.jpg'];
    I = imread([folderNameAdi fileNameAdi]);
    Z = imbinarize(I, 0.5);
    
    [numChambers, chamberCenters, logMedImg, wtrShdMskImg, bkgroundMsk, binImage, binImage1] = num_chamber_detect(prob_map);
    
    wtrShdMskModefilImg = colfilt(wtrShdMskImg, [10,10], 'sliding', @mode);
    
    k=numChambers+1;
    
    tic;
    X = double(wtrShdMskModefilImg);
    mu = double(unique(X));
    
    sigma = sqrt(2)*(mu(2)-mu(1))*ones(k, 1);
    sumUMat = zeros(kDeltaInit, numCircshift);
    kMat = zeros(kDeltaInit, numCircshift);
    corrMat = zeros(kDeltaInit, numCircshift);
    
    % construct prob_map_shifted. This will be used in future for correlation.
    epmNobkgrndPxl = prob_map(bkgroundMsk>0);
    prob_mapMskVec = prob_map(bkgroundMsk>0);
    prob_map_shifted = double(prob_map)-mean(double(prob_mapMskVec(:)));
    prob_map_shifted = prob_map_shifted.*(bkgroundMsk>0);
    
    kDelta = min(k, kDeltaInit);
    
    for kInd = 1:kDelta% different k values
        for indCC = 1:numCircshift% different cyclic shifts
            [Xout, ~, ~, sumUVec]=HMRF_EM(X,X,Z,mu,sigma,k,EM_iter,MAP_iter);
            XoutAll{kInd, indCC} = Xout;
            
            sumUMat(kInd, indCC) = sumUVec(end); % might be obsolete. Consider deletion [TODO]
            kMat(kInd, indCC) = k;
            % get correlation value for each of the candidates
            [corrVal] = get_correlation_value(Xout, prob_map_shifted, bkgroundMsk);
            corrMat(kInd, indCC) = corrVal;
            [X, mu] = circshift_labels(X, mu);
        end
        k = k-1;
    end
    
    [maxCorrVal, maxCorrPos] = max(corrMat(:));
    XoutBest = XoutAll{maxCorrPos};
    
    % get covering metric for HMRF
    [coveringMetricHMRF] = get_covering_metric(label_im, XoutBest);
    coveringMetricHMRFAll(indSamp) = coveringMetricHMRF;
    
    % get covering metric for watershed only
    [coveringMetricWtrShd] = get_covering_metric(label_im, double(wtrShdMskModefilImg));
    coveringMetricWtrShdAll(indSamp) = coveringMetricWtrShd;
    
    save(['simRes/' fileName 'kDel' num2str(kDeltaInit) 'numCC' num2str(numCircshift) '.mat'],'coveringMetricHMRF','coveringMetricWtrShd','wtrShdMskModefilImg','XoutBest');
end

%% plotting



