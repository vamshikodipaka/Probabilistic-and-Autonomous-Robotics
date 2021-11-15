function q = twoDmove(p,u,v)
%% 2D-MOVE FUNCTION WITH INACCURACY INCLUDED
% Using CIRCSHIFT for circular movement with associated error
%%
%Taking uncertainities::
%pCorrect = 0.8; %pOverShoot = 0.1; % pUnderShoot = 0.1;
q = p;
% Put u or v not equals to zero
% which means robot certainly make a move (atleast once)
% either in horizontal or in vertical direction
if v == 0
    
%   Next 3 lines consistute to horizontal move
    qu = 0.8 * circshift(p,[0,u]);    % if pCorrect
    qv = 0.1 * circshift(p,[0,u+1]);  % if pOverShoot
    qw = 0.1 * circshift(p,[0,u-1]);  % if pUnderShoot
    q = qu + qv + qw;       % computing q after motion Inaccuracy
else
%   Next 3 lines consistute to vertical move
    qu = 0.8 * circshift(p,[v,0]);     % if pCorrect
    qv = 0.1 * circshift(p,[v+1,0]);   % if pOverShoot
    qw = 0.1 * circshift(p,[v-1,0]);   % if pUnderShoot
    q = qu + qv + qw;       % computing q after motion Inaccuracy
end
end
%%FUNCTION END HERE