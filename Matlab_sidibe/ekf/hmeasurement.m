function [ y ] = hmeasurement( x,t )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

if t < 30
    y = 0.2 * (x.^2);
else
    y = 0.5 * x;
end

end

