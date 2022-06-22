%% a* algorithm
% YU CHUANGYANG  Waseda University
% Code for Robot Path Planning using A* algorithm
%% 

function h=historic(a,b)
h = sqrt(sum((a-b).^2));