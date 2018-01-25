function C = fnc_CoveringMetric(St,Sp)
% This function computes the covering metric between the groundtruth
% segmentation St and the proposed segmentation Sp.
% 
% Author: Edgar Lobaton
% Created: 11/26/2017

% Getting IDs of regions
St_ID = unique(St);
St_ID = St_ID(St_ID>0);
Sp_ID = unique(Sp);
Sp_ID = Sp_ID(Sp_ID>0);

% Initializing covering metric
C = 0;
% Initializing total count of pixels in the groundtruth segmentation
Ns = 0;

% Iterating over all regions in the groundtruth segmentation 
for i = 1:length(St_ID)
    % Extracting region
    Rt = (St==St_ID(i));
    
    % Initializing overlap ratio vector
    O = zeros(1,length(Sp_ID));
    
    % Iterating over all regions in the proposed segmentation
    for j = 1:length(Sp_ID)
        % Extracting region
        Rp = (Sp==Sp_ID(j));
        % Computing overlap ratio
        O(j) = nnz(Rt&Rp)/nnz(Rt|Rp);
    end
    
    % Updating covering metric
    Nr = nnz(Rt);
    C = C + Nr*max(O);
    
    % Updting total count of pixels in groundtruth segmentaiton
    Ns = Ns + Nr;
end

% Normalizing covering metric
C = C/Ns;
