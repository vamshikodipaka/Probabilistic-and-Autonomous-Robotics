function q = twoDsense(p, z, world)
%% 2D-SENSE FUNCTION WITH INACCURACY INCLUDED
% for 'p' distributed probabilities (most commonly 'p' of uniform prob)
% iterate through the world and compare z measurements
% compute correct and incorrect sensing measurements

%% Function starts here
[row,col] = size(world);

% Let z iterate through world
for ii= 1:row
    for jj= 1:col
        if strcmp(world(ii,jj),z)
            q(ii,jj) = p(ii,jj) * 0.6;  %correct measurement
        else
            q(ii,jj) = p(ii,jj) * 0.2; %incorrect measurement
        end
    end
end

%  normalizing all computed probabilities(of sensed measurements)
const = 1/sum(sum(q));
q = q .* const;
%  p = q;
end
%%FUNCTION END HERE