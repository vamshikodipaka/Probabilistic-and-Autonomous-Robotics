function err= costFuncPlaneInf( PPM, w, n, lambda )
% optimization funcn to get plane at infinity
err = [];

for i=1:size(PPM,3)
   leftpar = PPM(:,:,i) * [w, w*n ; n'*w, n'*w*n ] * PPM(:,:,i)';
   rightpar = lambda * w;
   errorTemp = leftpar-rightpar;
   err = [err errorTemp(:)'];
end
end

