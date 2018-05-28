function [roi, pxlCoordAll, pxlIdxAll] = get_region_of_interest(X, numLabels, centroids)

numLabels = unique(X)
imgSize = size(X, 1);


% Apply column filter to remove inter-chamber separation.
% Region growing
%wtrShdMskModefilImg = colfilt(wtrShdMskImg, [10,10], 'sliding', @mode);
%wtrShdMskModefilImg = min(bkgroundMsk, wtrShdMskModefilImg);
%figure(1);imagesc(wtrShdMskModefilImg)

% refine mask. Mask gets affected after col filt - mode filtering
%bkgroundMsk = double((bkgroundMsk>0)&(wtrShdMskModefilImg>0)); 
numPxlOfInterest = nnz(X);
pxlCoordAll = zeros(numPxlOfInterest, 2);
pxlIdxAll = zeros(1, numPxlOfInterest);
coordCnt= 0;

% % testing centroids
% aa = zeros(imgSize);
% for indLabel = 1:numLabels
%     aa(centroids(indLabel, 1), centroids(indLabel, 2)) = 1;
% end
% figure;imagesc(aa);

for indLabel = 1:numLabels
    labelVal = wtrShdMskModefilImg(centroids(indLabel, 1), centroids(indLabel, 2));
    roi{indLabel}.pxlIdx = find(wtrShdMskModefilImg == labelVal);
    roi{indLabel}.pxlCoord(:, 1) = mod(roi{indLabel}.pxlIdx-1, imgSize)+1;
    roi{indLabel}.pxlCoord(:, 2) = floor((roi{indLabel}.pxlIdx-1)/imgSize)+1;
    roi{indLabel}.numPxl = length(roi{indLabel}.pxlIdx);
    pxlCoordAll(coordCnt+[1:roi{indLabel}.numPxl], :) = roi{indLabel}.pxlCoord;
    pxlIdxAll(coordCnt+[1:roi{indLabel}.numPxl]) = roi{indLabel}.pxlIdx;
    coordCnt=coordCnt+roi{indLabel}.numPxl;
end

% aa = zeros(imgSize);
% bb = zeros(imgSize);
% bb(roi{1}.pxlIdx) = 1;
% for indl = 1:numLabels
% for ind = 1:length(roi{indl}.pxlIdx)
%     aa(roi{indl}.pxlCoord(ind, 1), roi{indl}.pxlCoord(ind, 2)) = indl;
%     aa(centroids(indl,1), centroids(indl,2)) = 10;
% end
% end
% 
% 
% figure;imagesc(aa);
brkpnt1 = 1;
