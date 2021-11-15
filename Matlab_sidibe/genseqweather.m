function out = genseqweather(M, N, init)

for i=1:N
    if i == 1
        outseq{i} = M * init';
    else
        outseq{i} = M * outseq{i-1};
    end
end
   
outseq = cell2mat(outseq);

num = rand();

for i = 1:N
    temp = outseq(:,i);
    if num <= temp(1);
        out(i) = 'S';
    elseif num <= (temp(1)+temp(2))
        out(i) = 'C';
    else
        out(i) = 'R';
    end
end
    
    
