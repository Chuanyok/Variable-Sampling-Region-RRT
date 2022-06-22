%% RRT algorithm
% YU CHUANGYANG  Waseda University
% Code for Robot Path Planning using Rapidly-exploring Random Trees
%% 
map=im2bw(imread('Maps/23279020_15.tif')); % input map read from a bmp file. for new maps write the file name here
map2=im2bw(imread('Maps/23279020_15.tif')); 

image((map==0).*0 + (map==1).*255);
axis image
colormap(gray(256))

source=[ 1390 22 ]; % source position in Y, X format
goal=[ 680 990 ]; % goal position in Y, X format

% disp('select source in the image');
% [x,y] = ginput(1);
% source=[double(int16(y)) double(int16(x))];    % source position in Y, X format
% 
% disp('select goal in the image');
% [x,y] = ginput(1);
% goal = [double(int16(y)) double(int16(x))];    % goal position in Y, X format

Gb=1;% set goalbias
goalbias=Gb; % set goalbias
loc=5;
max=5; % set size of each step of the RRT
disTh=2; % nodes closer than this threshold are taken as almost the same

maxFailedAttempts = 1000000;
display=true; % display of RRT
rectanglesize=size(map);
rectanglesize=[rectanglesize(2) rectanglesize(1)];
%%%%% parameters end here %%%%%

tic;
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display, imshow(map);rectangle('position',[1, 1, rectanglesize-1],'edgecolor','k'); end
RRTree=double([source -1]); % RRT rooted at the source, representation node and parent index
failedAttempts=0;
counter=0;
pathFound=false;
newPoint=source;

while failedAttempts<=maxFailedAttempts  % loop to grow RRTs
    if rand > goalbias 
        sample=rand(1,2) .* 10 + newPoint - 5; % random sample
        if sample == newPoint
            continue;
        end
        stepsize=loc;
    else
        stepsize=max;
        sample=goal; % sample taken as goal to bias tree generation to goal
    end
    [A, I]=min( distanceCost(RRTree(:,1:2),sample) ,[],1); % find closest as per the function in the metric node to the sample
    
    closestNode = RRTree(I(1),1:2);
    previoustheta=theta;
    theta=atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  % direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts=failedAttempts+1;
        goalbias=0;
        continue;
    end

    RRTree=[RRTree;newPoint I(1)]; % add node
    
    if show
        line([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)],'Color','blue','LineWidth',1.5);
        counter=counter+1;M(counter)=getframe;
    end  
    
    if distanceCost(newPoint,goal)<=max && checkPathtoGoal(newPoint,goal,map), pathFound=true;break; end % goal reached
    goalbias=Gb;
    
end

% if pathFound
%     line([newPoint(2);goal(2)],[newPoint(1);goal(1)],'Color','blue','LineWidth',1.5);
%     counter=counter+1;M(counter)=getframe;
% end 

if ~pathFound, error('no path found. maximum attempts reached'); end
path=[newPoint;goal];
prev=I(1);
while prev>0
    path=[RRTree(prev,1:2);path];
    prev=RRTree(prev,3);
end

path_optimized=path;
pathsize=size(path,1);
start_node=1;
curcheck_node=pathsize;

while start_node<size(path_optimized,1)-1 
        
    while curcheck_node-start_node>1
        
        if checkPathFast(path_optimized(curcheck_node,1:2), path_optimized(start_node,1:2), map,loc)          
            id_X=path_optimized(curcheck_node,2);
            id_Y=path_optimized(curcheck_node,1);      
            path_optimized(start_node+1:curcheck_node-1,:) =[];     
            start_node=find(path_optimized(:,1)==id_Y & path_optimized(:,2)==id_X);
            curcheck_node=size(path_optimized,1);
            break;
        else
            curcheck_node=curcheck_node-1;
            if ~(curcheck_node-start_node>1)
                start_node=start_node+1;
                curcheck_node=size(path_optimized,1);
            end
        end 
    end      
end

fprintf('processing time=%d \n', toc);

% if display 
%     disp('press any key');
%     waitforbuttonpress; 
% end

pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('Path Length=%d \n', pathLength); 
% imshow(map);rectangle('position',[1 1 rectanglesize-1],'edgecolor','k');
line(path(:,2),path(:,1),'Color','red','LineWidth',2.5);

if display 
    disp('press any key');
    waitforbuttonpress; 
end

pathLength_optimiazed=0;
for i=1:length(path_optimized)-1, pathLength_optimiazed=pathLength_optimiazed+distanceCost(path_optimized(i,1:2),path_optimized(i+1,1:2)); end
fprintf('optimized Path Length=%d \n', pathLength_optimiazed); 
f = figure;
imshow(map);rectangle('position',[1 1 rectanglesize-1],'edgecolor','k');
line(path_optimized(:,2),path_optimized(:,1),'Color','blue','LineWidth',2.5);

f2 = figure;
imshow(map);rectangle('position',[1 1 rectanglesize-1],'edgecolor','k');
line(path(:,2),path(:,1),'Color','red','LineWidth',2.5);
line(path_optimized(:,2),path_optimized(:,1),'Color','blue','LineWidth',2.5);