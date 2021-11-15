function [mew sigma] = update(mew1,sigma1,mew2,sigma2)

mew = (sigma2 * mew1 + sigma1 * mew2)/(sigma1+ sigma2);
sigma = (sigma1 * sigma2) /(sigma1 + sigma2);

end
