folderName = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\foram - training set\';
savefolderName = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\TrainingSetRefined\';
savefolderNameMat = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\TrainingSetRefinedMat\';
savefolderNameCsv = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\TrainingSetRefinedCsv\';

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
    
     imwrite(prob_map, [savefolderName savefileName '.jpg']);
     imwrite(uint8(round(label_im/max(label_im(:))*255)), [savefolderName savefileName '_label.jpg']);
    
    label_im = round(label_im/max(label_im(:))*255);
    
     save([savefolderNameMat savefileName '.mat'], 'prob_map');
     save([savefolderNameMat savefileName '_label.mat'], 'label_im');
    
%     csvwrite([savefolderNameCsv savefileName '.csv'], prob_map);
%     csvwrite([savefolderNameCsv savefileName '_label.csv'], label_im);
end