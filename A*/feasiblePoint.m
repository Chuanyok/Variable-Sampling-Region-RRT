%% a* algorithm
% YU CHUANGYANG  Waseda University
% Code for Robot Path Planning using A* algorithm
%% 

function feasible=feasiblePoint(point,map)
feasible=true;
% check if collission-free spot and inside maps
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
    feasible=false;
end