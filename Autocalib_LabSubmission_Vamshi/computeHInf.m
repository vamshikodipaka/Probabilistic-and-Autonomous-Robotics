function HInfi = computeHInf(Fs,planeInfi)
% this function to compute HInfi
k = 1;
for i = 2:size(Fs,3)
    for j = i+1:size(Fs,3)
        % for HInfi we have below eq.
        HInfi{k} =  epipole(Fs(:,:,i,j)) * Fs(:,:,i,j) + epipoleVect(Fs(:,:,i,j)) * planeInfi';
        k = k+1;
    end
end        
end

% Finding epipoleVec -------------
function epi = epipoleVect( F )
% finding epipole by svd
[U S V]  = svd(F');
epi = V(:,end);
end
