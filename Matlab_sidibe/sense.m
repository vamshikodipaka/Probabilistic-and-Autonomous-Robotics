function q = sense(p, z, world)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% input
%
% p               prior
% z               measurement
% world
%
% output
%
% q  posterior distribution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_hit = 0.6;
p_miss = 0.2;
% for j = 1:length(z)
    for i= 1:length(world)
        if strcmp(world{i},z)
            p(i) = p_hit * p(i);
        else
            p(i) = p_miss * p(i);
        end
        
    end
    
    norm_const = sum(p);
    
    p = p ./ norm_const;
    
    %     p = p_tilda
    
% end

q = p;

