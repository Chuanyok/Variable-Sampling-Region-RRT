%% RRT algorithm
% YU CHUANGYANG  Waseda University
% Code for Robot Path Planning using Rapidly-exploring Random Trees
%% 

function feasible=checkPathFast(n,newPos,map,stepsize)

feasible=true;
dir=atan2(newPos(1)-n(1),newPos(2)-n(2));
t=fix(sqrt(sum((n-newPos).^2))/stepsize);

for r=stepsize:stepsize:stepsize*t
    posCheck=n+r.*[sin(dir) cos(dir)];
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ... 
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;return;
    end
%     if ~feasiblePoint(newPos,map), feasible=false; end
end

for r=0.5:0.5:sqrt(sum((n-newPos).^2))
    posCheck=n+r.*[sin(dir) cos(dir)];
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ... 
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;return;
    end
%     if ~feasiblePoint(newPos,map), feasible=false; end
end