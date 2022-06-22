%% RRT algorithm
% YU CHUANGYANG  Waseda University
% Code for Robot Path Planning using Rapidly-exploring Random Trees
%% 
map=im2bw(imread('Maps/23279020_15.tif')); % input map read from a bmp file. for new maps write the file name here
map2=imread('Maps/23279020_15.tif');

% image((map==0).*0 + (map==1).*255);
% axis image
% colormap(gray(256))

% source=[ 740 20 ]; % source position in Y, X format 
% goal=[ 20 880 ]; % goal position in Y, X format

source=[ 1390 22 ]; % source position in Y, X format 23279020 scenario1
goal=[ 680 990 ]; % goal position in Y, X format

% source=[ 1360 1250 ]; % source position in Y, X format scenario2
% goal=[ 155 145 ]; % goal position in Y, X format

% source=[ 150 1360 ]; % source position in Y, X format 23129050 scenario3
% goal=[ 1120 110 ]; % goal position in Y, X format

% source=[ 760 1290 ]; % source position in Y, X format 23129050 scenario4
% goal=[ 180 420 ]; % goal position in Y, X format

% source=[ 1210 320 ]; % source position in Y, X format 23729065 scenario5
% goal=[ 180 960 ]; % goal position in Y, X format 

% source=[ 1210 1040 ]; % source position in Y, X format 23729065 scenario6
% goal=[ 180 960 ]; % goal position in Y, X format

a=5; % set stepsize and disTh
goalbias=0.9; % set goalbias

stepsize=a; % size of each step of the RRT
disTh=2; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000000;
display=true; % display of RRT
show=false;
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

while failedAttempts<=maxFailedAttempts  % loop to grow RRTs
    if rand > goalbias 
        sample=rand(1,2) .* size(map); % random sample
    else
        sample=goal; % sample taken as goal to bias tree generation to goal
    end
    [A, I]=min( distanceCost(RRTree(:,1:2),sample) ,[],1); % find closest as per the function in the metric node to the sample
    closestNode = RRTree(I(1),1:2);
    theta=atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  % direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts=failedAttempts+1;
        continue;
    end

%     [A, I2]=min( distanceCost(RRTree(:,1:2),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
%     if distanceCost(newPoint,RRTree(I2(1),1:2))<disTh, continue; end % failedAttempts=failedAttempts+1;continue; end 
    RRTree=[RRTree;newPoint I(1)]; % add node
    
    if show
        line([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)],'Color','blue','LineWidth',1.5);
        counter=counter+1;M(counter)=getframe;
    end
    
    if distanceCost(newPoint,goal)<=a && checkPath(newPoint,goal,map), pathFound=true;break; end % goal reached
end

if pathFound && show
    line([newPoint(2);goal(2)],[newPoint(1);goal(1)],'Color','blue','LineWidth',1.5);
    counter=counter+1;M(counter)=getframe;
end

if ~pathFound, fprintf('processing time=%d \n', toc);error('no path found. maximum attempts reached'); end
path=[newPoint;goal];
prev=I(1);

while prev>0
    path=[RRTree(prev,1:2);path];
    prev=RRTree(prev,3);
end

fprintf('processing time=%d \n', toc);
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('path length=%d \n', pathLength); 
imshow(map);rectangle('position',[1 1 rectanglesize-1],'edgecolor','k');
line(path(:,2),path(:,1),'Color','red','LineWidth',2.5);

disp('press any key');
waitforbuttonpress;
f=figure;
imshow(map2);rectangle('position',[1 1 rectanglesize-1],'edgecolor','k');
line(path(:,2),path(:,1),'Color','red','LineWidth',2.5);