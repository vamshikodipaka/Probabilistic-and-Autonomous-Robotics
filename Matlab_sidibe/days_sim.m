function [seq] = days_sim(M,N,init)

seq = [];

for i=1:N
    next = M * init;
    
    f = rand();
    
    if f <= next(1)
        seq(i) = 's';
    elseif f <= (next(1)+next(2))
        seq(i) = 'c';
    else
        seq(i) = 'r';
    end
    
    init = next;
end
        