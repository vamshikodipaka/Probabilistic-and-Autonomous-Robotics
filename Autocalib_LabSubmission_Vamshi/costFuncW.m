function err = costFunctionW( HInfi, p )
% %this cost function minimizes w 

A=[p(1) p(2) p(3); 0 p(4) p(5); 0 0 1];
% we have ----
w = A * A';
err = [];

for i = 1:length(HInfi)
   leftpar =   HInfi{i} * w * HInfi{i}';
   errorTemp = leftpar - w;
   err = [err errorTemp(1,:) errorTemp(2,2:3)] ;
   
end
end

