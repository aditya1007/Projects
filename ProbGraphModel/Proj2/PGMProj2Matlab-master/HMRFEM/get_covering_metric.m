function [coveringMetric] = get_covering_metric(gt, xout)

imgSize = size(gt, 1); % assumption - square image

labelTrainList = unique(gt);
labelTestList = unique(xout);
labelTrainList(labelTestList == 0) = [];
labelTestList(labelTestList == 0) = [];

numLabelTrain = length(labelTrainList);
numLabelTest = length(labelTestList);

trainMsk = false(imgSize, imgSize, numLabelTrain);
for ind = 1:numLabelTrain
    trainMsk(:,:,ind) = (gt == labelTrainList(ind)); % R
end

testMsk = false(imgSize, imgSize, numLabelTest);
for ind = 1:numLabelTest
    testMsk(:,:,ind) = (xout == labelTestList(ind)); % R^prime
end

% construct OMat
maxOVec = zeros(1, numLabelTrain);
for indTrain = 1:numLabelTrain
    OVec = zeros(1, numLabelTest);
    trainMskTmp = trainMsk(:,:,indTrain);
    for indTest = 1:numLabelTest
        numIntersect = nnz(trainMskTmp&testMsk(:, :, indTest));
        numUnion = nnz(trainMskTmp|testMsk(:,:,indTest));
        OVec(indTest) = numIntersect/numUnion;
    end
    maxOVec(indTrain) = nnz(trainMskTmp)*max(OVec);
end

coveringMetric = sum(maxOVec)/nnz(gt~=0);




