folderName = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\foram - training set\';
savefolderNameMat = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\TrainingSetRefinedMat\';

samplesList = 1:455;
cntVal = 0;

for indSamp = 1:length(samplesList)
    sampleNum  = samplesList(indSamp);
    if sampleNum < 10
        fileName = ['00' num2str(sampleNum) '_sample'];
    elseif sampleNum < 100
        fileName = ['0' num2str(sampleNum) '_sample'];
    else
        fileName = [num2str(sampleNum) '_sample'];
    end
    if ~exist([folderName fileName '.mat'], 'file')
        continue;
    end
    
    cntVal = cntVal+1;
    
    if cntVal < 10
        savefileName = ['00' num2str(cntVal) '_sample'];
    elseif cntVal < 100
        savefileName = ['0' num2str(cntVal) '_sample'];
    else
        savefileName = [num2str(cntVal) '_sample'];
    end
    
    load([folderName fileName '.mat']);
    
    % edge detection
    [Gx, Gy] = imgradientxy(label_im);
    [Gmag, Gdir] = imgradient(Gx, Gy);
    
    edge_label_im = imbinarize(Gmag,2);
    edge_label_im = im2uint8(edge_label_im);
    
    save([savefolderNameMat savefileName '_edge_label.mat'], 'edge_label_im');
end