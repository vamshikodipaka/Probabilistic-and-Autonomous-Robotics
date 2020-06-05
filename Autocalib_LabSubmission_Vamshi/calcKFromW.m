function K = calcKFromW( w )
%   Calculating K from w    ------------
kk1 = w(1);kk2 = w(2);kk3 = w(3);kk4 = w(4);kk5 = w(5);
au = sqrt(kk1 - (kk2 - kk3 * kk5)^2/(kk4 - kk5^2) - kk3^2);
gamma = (kk2 - kk3 * kk5)^2/sqrt(kk4 - kk5^2);

u0 = kk3;   v0 = kk5;

av = sqrt(kk4 - kk5^2);
K = [au gamma u0; 0 av v0; 0 0 1];  
end

