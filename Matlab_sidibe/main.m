clc;
z = [ 5 6 7 9 10 ];
u = [1 1 2 1 1];
sigma2_mea = 4;
sigma2_mot = 2;

mew1 = 0;
sigma1 = 10000;

for i = 1:5
    disp(['iteration ',num2str(i), ':']);
    [mew, sigma] = update(mew1, sigma1, z(i), sigma2_mea);
    disp(['update: ','mu = ', num2str(mew), ' and ', 'sigma = ',num2str(sigma)]);
    %mew1 = mew; sigma1 = sigma;
    
    [mew, sigma] = predict(mew, sigma, u(i), sigma2_mot);
    disp(['predict: ','mu = ', num2str(mew), ' and ', 'sigma = ',num2str(sigma)]);
    mew1 = mew; sigma1 = sigma;
end
