function [mew sigma] = predict(mew1,sigma1,mew2,sigma2)

mew = mew1 + mew2;
sigma = sigma1 + sigma2;

end
