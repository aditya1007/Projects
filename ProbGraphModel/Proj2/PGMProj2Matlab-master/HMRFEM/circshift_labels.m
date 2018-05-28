function [Xout, muOut] = circshift_labels(X, mu)

kVal = length(mu);
muOut(1:kVal-1) = mu(2:kVal);muOut(kVal) = mu(1); 
%sigmaOut(1:kVal-1) = sigma(2:kVal);sigmaOut(kVal) = sigma(1);

labelList = unique(X);
labelList(labelList == 0) = [];
numLabels = length(labelList);

Xout = zeros(size(X));

% record pixel indices for each label
for ind = 1:numLabels
    pxlIdx = find(X == labelList(ind));
    if ind<numLabels
        Xout(pxlIdx) = labelList(labelList(ind+1));
    else
        Xout(pxlIdx) = labelList(labelList(1));
    end
end



