function [y] = fstate( x )
%This function to get state
y = 1 + sin(4*1e-2*pi*x) + 0.5 * x;
end

