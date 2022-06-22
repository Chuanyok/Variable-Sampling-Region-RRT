%% a* algorithm
% YU CHUANGYANG  Waseda University
% Code for Robot Path Planning using A* algorithm
%% 

function [final_path, final_path_found] = a_start_compute_path(map, ~, source, goal, display_process, resolutionX, resolutionY)

    if(display_process == true )
        figure(2);
        colormap(gray(256));
        subplot(1,1,1);
    end

    % Conection matrix - define admisible movement of robot
    conn=[1 1 1;
          1 2 1;
          1 1 1];
  
    %structure of a node is taken as positionY, positionX, historic cost, heuristic cost, total cost, parent index in closed list (-1 for source) 
    Q=[source 0 heuristic(source,goal) 0+heuristic(source,goal) -1]; % the processing queue of A* algorihtm, open list
    closed=ones(size(map)); % the closed list taken as a hash map. 1=not visited, 0=visited
    closedList=[]; % the closed list taken as a list
    final_path_found=false;

    counter=0;
    
    while size(Q,1)>0
         [A, I]=min(Q,[],1);
         n=Q(I(5),:); % smallest cost element to process
         Q=[Q(1:I(5)-1,:);Q(I(5)+1:end,:)]; % delete element under processing
         if n(1)==goal(1) && n(2)==goal(2) % goal test
             final_path_found=true;break;
         end
         
         [rx,ry,rv]=find(conn==2); % robot position at the connection matrix
         [mx,my,mv]=find(conn==1); % array of possible moves
         for mxi=1:size(mx,1) %iterate through all moves
             newPos=[n(1)+mx(mxi)-rx n(2)+my(mxi)-ry]; % possible new node
             if checkPath(n(1:2),newPos,map) %if final_path from n to newPos is collission-free
                  if closed(newPos(1),newPos(2))~=0 % not already in closed
                      historicCost=n(3)+historic(n(1:2),newPos);
                      heuristicCost=heuristic(newPos,goal);
                      totalCost=historicCost+heuristicCost;
                      add=true; % not already in queue with better cost
                      if length(find((Q(:,1)==newPos(1)) .* (Q(:,2)==newPos(2))))>=1
                          I=find((Q(:,1)==newPos(1)) .* (Q(:,2)==newPos(2)));
                          if Q(I,5)<totalCost, add=false;
                          else Q=[Q(1:I-1,:);Q(I+1:end,:);];add=true;
                          end
                      end
                      if add
                          Q=[Q;newPos historicCost heuristicCost totalCost size(closedList,1)+1]; % add new nodes in queue
                      end
                  end
             end           
         end
         
         closed(n(1),n(2))=0;closedList=[closedList;n]; % update closed lists
         i0 = counter;
         i1 = 40;
         counter=counter+1;

         if display_process == true && (rem(i0,i1) == 0) 
             temp_img = (map==0).*0 + ((closed==0).*(map==1)).*125 + ((closed==1).*(map==1)).*255;

             % plot goal and source
             temp_img(goal(1), goal(2) ) = 160;
             temp_img(source(1), source(2) ) = 160;

             image(temp_img);

            M(counter)=getframe;
            
            %pause(1);
         end
    end

    if ~final_path_found
        disp('no final_path found')
    end

    final_path=[n(1:2)]; %retrieve final_path from parent information
    prev=n(6);
    while prev>0
        final_path=[closedList(prev,1:2);final_path];
        prev=closedList(prev,6);
    end
end