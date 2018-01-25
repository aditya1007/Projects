% Foraminifera test
folderNameTest = 'ProbMap\';
maxSampIdxList = 470;
sampleNum = 1;
sampleCnt = 0;

for ind = 1:maxSampIdxList
    
    sampleNum = ind;
    
    if sampleNum < 10
        fileNameTest = ['00' num2str(sampleNum) '_ProbMap.mat'];
    elseif sampleNum < 100
        fileNameTest = ['0' num2str(sampleNum) '_ProbMap.mat'];
    else
        fileNameTest = [num2str(sampleNum) '_ProbMap.mat'];
    end
    
    if ~exist([folderNameTest fileNameTest], 'file')
        continue;
    end
    
    load([folderNameTest fileNameTest]);
    
    sampleCnt = sampleCnt+1;
    
    %% detecting the number of chambers and centroids
    [numChambers, chamberCenters, logMedImg, wtrShdMskImg, bkgroundMsk] = num_chamber_detect(prob_map);
    centroids = fliplr(chamberCenters);
    
    % mode filter
    wtrShdMskModefilImg = colfilt(wtrShdMskImg, [10,10], 'sliding', @mode);
    wtrShdMskModefilImg = min(bkgroundMsk, wtrShdMskModefilImg); % Masking again to prevent extra growth (check performance by considering it)
    
    %coveringMetric = get_covering_metric
%    [coveringMetric] = get_covering_metric(label_im, wtrShdMskModefilImg);
%    coveringMetricAll(indSamp) = coveringMetric;

    if sampleNum < 10
        fileNamePred = ['00' num2str(sampleNum) '_sample.mat'];
    elseif sampleNum < 100
        fileNamePred = ['0' num2str(sampleNum) '_sample.mat'];
    else
        fileNamePred = [num2str(sampleNum) '_sample.mat'];
    end

    yPred = wtrShdMskModefilImg;
    save(['testPred\' fileNamePred], 'yPred');
    
    %figure(1);subplot(121);imagesc(yPred);subplot(122);imagesc(prob_map);
    
end
