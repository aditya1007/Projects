
folderName = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\foram - training set\';

%numSamples = 455;
%samplesList = 58; %1:numSamples;
samplesList = 1:455;

for indSamp = 1:length(samplesList)
    sampleNum  = samplesList(indSamp);
    if sampleNum < 10
        fileName = ['00' num2str(sampleNum) '_sample.mat'];
    elseif sampleNum < 100
        fileName = ['0' num2str(sampleNum) '_sample.mat'];
    else
        fileName = [num2str(sampleNum) '_sample.mat'];
    end
    if ~exist([folderName fileName], 'file')
        continue;
    end 
    
    load([folderName fileName]);
    figure(1);
    subplot(321)
    %figure(1);
    imshow(prob_map, [0 255]);
    title('a. Probability Map');
    subplot(322);
    %figure(2);
    imshow(label_im, [0 6]);
    title('b. Labeled Image');
    
    % thresholding
    T = adaptthresh(prob_map, 0.01);
    minT = 0.6;
    T(T<minT) = minT;
    binImage = imbinarize(prob_map, T);
    subplot(323);
    %figure(3);
    imshow(binImage, [0 1]);
    title('c. After Adaptive Thresholding');
    
    subplot(324)
    binImage1 = bwareaopen(binImage, 10);
    imshow(binImage1, [0 1]);
    title('d. After Small Object Removal');
    
    % distance transform
    distTranImage = bwdist(binImage1, 'euclidean');
    subplot(325);
    %figure(4);
    imshow(distTranImage, [0 35]);
    title('e. Distance Transform');
    
    % thresholding again
    binImage1 = imbinarize(distTranImage, 10);
    subplot(326);
    %figure(5);
    imshow(binImage1, [0 1]);
    title('f. Fixed Thresholding');
    
%     se = strel('disk',3);
%     closeImage = imclose(binImage, se);
%     figure(5);imshow(closeImage, [0 255]); 
%    pause(1);  
     pause;
end