folderName = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\TrainingSetRefinedMat\';
savefolderNameMat = 'C:\Users\Anuj Nayak\Google Drive\Documents\MATLAB\Sem3Fall2017\ProbabilisticGraphicalModeling\proj2\PatchSet\';

samplesList = 1:1000;
numPixelsPerImgAxis = 230;
numPixelsPerImg = numPixelsPerImgAxis^2;
patchSize = 32;

labelList = [];

for indSamp = 1:length(samplesList)
    sampleNum  = samplesList(indSamp);

    load([savefolderNameMat 'epmpatch' num2str(indSamp) '.mat']);    
    load([savefolderNameMat 'epmpatch_label' num2str(indSamp) '.mat']); 
    
    labelList = [labelList epm_patch_label];
    figure(1);imagesc(epm_patch);drawnow();
    pause(0.05);
end