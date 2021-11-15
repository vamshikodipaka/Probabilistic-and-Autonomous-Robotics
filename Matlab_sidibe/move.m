function q = move(p,u)

% if u == 0
%     q = p;
% elseif u == 1
%     q = circshift(p,ones(1,length(p)));
% else
%     q = circshift(p,-1*ones(1,length(p)));
% end

q = circshift(p',[u,0]);
q=q';


