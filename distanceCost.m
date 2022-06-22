%% RRT algorithm
% YU CHUANGYANG  Waseda University
% Code for Robot Path Planning using Rapidly-exploring Random Trees
%% 

function h=distanceCost(a,b)
h = sqrt((a(:,1)-b(:,1)).^2 + (a(:,2)-b(:,2)).^2 );