folderName = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\TrainingSetRefinedMat\';
savefolderNameMat = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\PatchSet\';

samplesList = 1:20;
cntVal = 0;
numPixelsPerImgAxis = 230;
numPixelsPerImg = numPixelsPerImgAxis^2;
patchSize = 20;
numEdgePatch = 10;
numNonEdgePatch = 20;
rand('seed', 0);randn('seed', 0);

for indSamp = 1:length(samplesList)
    sampleNum  = samplesList(indSamp);
    %% Load sample and edge labeled files
    if sampleNum < 10
        fileName = ['00' num2str(sampleNum) '_sample'];
        edgefilename = ['00' num2str(sampleNum) '_sample_edge_label'];
    elseif sampleNum < 100
        fileName = ['0' num2str(sampleNum) '_sample'];
        edgefilename = ['0' num2str(sampleNum) '_sample_edge_label'];
    else
        fileName = [num2str(sampleNum) '_sample'];
        edgefilename = [num2str(sampleNum) '_sample_edge_label'];
    end
    
    load([folderName fileName '.mat']);
    load([folderName edgefilename '.mat']);
    
    %% enlarge edge probability map
    epm_large = uint8(zeros(patchSize+numPixelsPerImgAxis, patchSize+numPixelsPerImgAxis));
    leftOffset = patchSize/2-1;
    topOffset = patchSize/2-1;
    epm_large(leftOffset+[1:numPixelsPerImgAxis], topOffset+[1:numPixelsPerImgAxis]) = prob_map;
    
    %% get edge and non-edge indices
    edgeIndices = find(edge_label_im == 255);
    nonedgeIndices = find(edge_label_im == 0);
    
    %% Randomly choose E edge indices and N non-edge indices
    randEdgeIndices = round(rand(1, 2*numEdgePatch)*length(edgeIndices));
    randEdgeIndices = unique(randEdgeIndices);
    randEdgeIndices = randEdgeIndices(1:numEdgePatch);
    randEdgeIndices = edgeIndices(randEdgeIndices);
    randNonEdgeIndices = round(rand(1, 2*numNonEdgePatch)*length(nonedgeIndices));
    randNonEdgeIndices = unique(randNonEdgeIndices);
    randNonEdgeIndices = randNonEdgeIndices(1:numNonEdgePatch);
    randNonEdgeIndices = nonedgeIndices(randNonEdgeIndices);
    
    pxlIndices = [randEdgeIndices;randNonEdgeIndices];
    
    randPxlCol = [floor((pxlIndices-1)/numPixelsPerImgAxis)+1];
    randPxlRow = [mod((pxlIndices-1),numPixelsPerImgAxis)+1];
    
    lblListDbg = [];
    
    for indPxl = 1:length(randPxlRow)
        rowShift = randPxlRow(indPxl);
        colShift = randPxlCol(indPxl);
        epm_patch = epm_large(rowShift+[1:patchSize], colShift+[1:patchSize]);
        epm_patch_label = edge_label_im(pxlIndices(indPxl));
        cntVal = cntVal+1;
        save([savefolderNameMat 'epmpatch' num2str(cntVal) '.mat'], 'epm_patch');
        save([savefolderNameMat 'epmpatch_label' num2str(cntVal) '.mat'], 'epm_patch_label');
        
        % >>>> plot for debug
        %figure(1);imagesc(epm_patch);
        %drawnow();
        %lblListDbg = [lblListDbg epm_patch_label];
        %pause(0.05);
    end
end



%% Old code
% folderName = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\TrainingSetRefinedMat\';
% savefolderNameMat = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\PatchSet\';
% 
% samplesList = 1:2;
% cntVal = 0;
% numPixelsPerImgAxis = 230;
% numPixelsPerImg = numPixelsPerImgAxis^2;
% patchSize = 20;
% numPatchesPerSamp = 10;
% rand('seed', 0);randn('seed', 0);
% 
% for indSamp = 1:length(samplesList)
%     sampleNum  = samplesList(indSamp);
%     if sampleNum < 10
%         fileName = ['00' num2str(sampleNum) '_sample'];
%         edgefilename = ['00' num2str(sampleNum) '_sample_edge_label'];
%     elseif sampleNum < 100
%         fileName = ['0' num2str(sampleNum) '_sample'];
%         edgefilename = ['0' num2str(sampleNum) '_sample_edge_label'];
%     else
%         fileName = [num2str(sampleNum) '_sample'];
%         edgefilename = [num2str(sampleNum) '_sample_edge_label'];
%     end
%     
%     load([folderName fileName '.mat']);
%     load([folderName edgefilename '.mat']);
%     
%     % enlarge edge probability map
%     epm_large = uint8(zeros(patchSize+numPixelsPerImgAxis, patchSize+numPixelsPerImgAxis));
%     leftOffset = patchSize/2-1;
%     topOffset = patchSize/2-1;
%     epm_large(leftOffset+[1:numPixelsPerImgAxis], topOffset+[1:numPixelsPerImgAxis]) = prob_map;
%     
%     % 1 indices
%     randPxlRow = round(rand(1, 2*numPatchesPerSamp)*numPixelsPerImgAxis);
%     randPxlRow = unique(randPxlRow);
%     randPxlRow = randPxlRow(1:numPatchesPerSamp);
%     randPxlCol = round(rand(1, 2*numPatchesPerSamp)*numPixelsPerImgAxis);
%     randPxlCol = unique(randPxlCol);
%     randPxlCol = randPxlCol(1:numPatchesPerSamp);
%     
%     for indPxlRow = 1:length(randPxlRow)
%         epm_patch = uint8(zeros(patchSize));
%         rowShift = randPxlRow(indPxlRow);
%         for indPxlCol = 1:length(randPxlCol)
%             colShift = randPxlCol(indPxlCol);
%             epm_patch = epm_large(rowShift+[1:patchSize], colShift+[1:patchSize]);
%             epm_patch_label = edge_label_im(indPxlRow, indPxlCol);
%             %epm_patch_label = 
%             cntVal = cntVal+1;
%             save([savefolderNameMat 'epmpatch' num2str(cntVal) '.mat'], 'epm_patch');
%             save([savefolderNameMat 'epmpatch_label' num2str(cntVal) '.mat'], 'epm_patch_label');
%         end
%     end
% end