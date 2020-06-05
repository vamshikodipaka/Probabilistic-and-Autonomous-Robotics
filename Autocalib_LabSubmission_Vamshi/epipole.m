function epi = epipole( F )

[M N O]  = svd(F');

epi = O(:,end);

% 
epi = [ 0 -epi(3) epi(2)
     epi(3) 0 -epi(1) 
    -epi(2) epi(1) 0];

end

