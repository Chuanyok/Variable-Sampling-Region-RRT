%% a* algorithm
% YU CHUANGYANG  Waseda University
% Code for Robot Path Planning using A* algorithm
%% 

function h=heuristic(X,goal)
h = sqrt(sum((X-goal).^2));