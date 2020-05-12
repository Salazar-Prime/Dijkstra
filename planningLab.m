% ME 597 - Autonomous Systems
% Planning Lab
% Template by Brian Page
% March 23, 2020

% Submission by: VARUN AGGARWAL
% 3/30/2020

clear
clc
close all

%% Create Maze

mazeSize = 10;
maze = zeros(mazeSize,mazeSize); % Maze is set up as empty space = 0, obstacle = 1, starting position = 2, target = 3

%set up prompts
axis([1 mazeSize+1 1 mazeSize+1]);
grid on
hold on
xlabel('X')
ylabel('Y')

%prompt for target location
title('Click for target location')
[tempx,tempy] = ginput(1);
target = [floor(tempx),floor(tempy)];
maze(target(1),target(2)) = 3;
plot(target(1)+0.5,target(2)+0.5,'ko')

%Prompt for obstacles
title('Left click for obstacles, right click to end');
button = 1;
obstacles = 0;
while button==1
    [tempx,tempy,button] = ginput(1);
    if button == 1
        obstacles = obstacles + 1;
        obstacle(:,obstacles) = [floor(tempx),floor(tempy)];
        maze(obstacle(1,obstacles),obstacle(2,obstacles)) = 1;
        plot(obstacle(1,obstacles)+0.5,obstacle(2,obstacles)+0.5,'rx')
    end
end

%Prompt for starting location
title('Left click starting location');
[tempx,tempy] = ginput(1);
starting = [floor(tempx),floor(tempy)];
maze(starting(1),starting(2)) = 2;
plot(starting(1)+0.5,starting(2)+0.5,'b*')

save('maze_v2.mat','maze')

%% Graph Setup

%First, build a graph of edge costs from the maze. Only allow travel to nearby blocks
%horizontal/vertical/diagonal motion and do not connect across obstacles
%. Effectively, you need to create an adjacency matrix that
%has edge weights from node to node.
clear
clc
close all

load('maze.mat')
% load('bigMaze.mat')
% load('maze_v2.mat')

%Now that we have our maze, we need to put it into a form that is solveable
%for planners.  There are lots of ways to do this, but one way is to set up
%a graph where each node in the graph is connected to nearby nodes. To do
%this, you will need to create an array of nodes (N x 3) where each
%node in the (N x N) maze is represented as a point in cartesian space. In
%the form [nodeID, x, y]

%node = [id X Y]


%YOUR CODE HERE

% nmuber of rows and columns in maze
maze_rows = length(maze(1,:));
maze_cols = length(maze(:,1));
index_node = 1;

% loop through every node in maze and assign it a unique ID
for i=1:maze_rows
    for j=1:maze_cols
        if maze(i,j) == 2
            start_ID = index_node;
        end
        if maze(i,j) == 3
            end_ID = index_node;
        end
        % for each node, find (x,y) and store in nodes array
        nodes(index_node, 1:3) = [index_node, i, j];
        index_node = index_node + 1; 
    end
end

%Now that we have our array of nodes, you will need to create an array of
%segments. These segments will form the graph. Basically, you need to
%figure out which nodes are able to connect to other nodes. Depending on
%what you want, you can make is so that only purely vertical/horizontal
%motion is allowed, or you could allow diagonal motion too. This array will
%be (M x 3) where M is number of segments in the form [segmentID, node1,
%node2]


%YOUR CODE HERE
clc
segments = [];
segmentCounter = 1;
for i=1:index_node-1
    % coordiante for each node
    x = nodes(i,2);
    y = nodes(i,3);
    
    % search for nodes around (x, y)
    for ii=x-1:x+1
        % skip rows which are out of bound eg. -ve and >10
        if ii < 1 || ii > maze_rows
            continue
        end
        for jj=y-1:y+1
            % to skip colums which are out of bound
            if jj < 1 || jj > maze_cols
                continue
            end
            % to skip adding edge for same cell 
            if ii == x && jj == y
                continue
            end
            % to skip obstacle cell 
            if maze(x,y) == 1
                continue
            end
            
            % skip diagonal
            if abs(ii - x) + abs(jj - y) == 2
                continue
            end
            
            % find segement
            if maze(ii,jj) ~= 1
                node_ID = intersect(find(nodes(:,2) == ii), find(nodes(:,3) == jj));
                segments(end+1,:) = [segmentCounter, nodes(i,1), node_ID];
                segmentCounter = segmentCounter + 1;
            end
        end
    end
end


%% Dijkstra

%Here is where you need to implement Dijstra's algorithm going from
%startpoint to endpoint. You will want to use the graph that was generated
%above to help solve the maze. The output of your algorithm should be
%cartesian distance and a vector path [1 x k] from node to node.

tic
%YOUR CODE HERE
[dist,path] = dijkstra(nodes, segments, start_ID, end_ID);
% [dist,path] = dj(nodes, segments, start_ID);
toc

%% plotting and animation

% Lets plot the nodes/segments
figure(1)
hold on
plot(nodes(:,2),nodes(:,3),'.k')
for ii = 1:segmentCounter-1
    plot(nodes(segments(ii,2:3)',2),nodes(segments(ii,2:3)',3),'k')
end

% plot start
plot(nodes(start_ID,2),nodes(start_ID,3),'.g', 'MarkerSize',20)
% plot end
plot(nodes(end_ID,2),nodes(end_ID,3),'.b', 'MarkerSize',20)

title(sprintf('Distance %0.4f', dist))

gif('myfile.gif','DelayTime', 1/5)

% video object
% v = VideoWriter('trial_movie.avi');
% v.FrameRate = 3;
% open(v);
% % set(gca,'Units','pixels','Position',[10 10 444 338])
% F = getframe;
% writeVideo(v,F);

%Plot the path
for ii = 1:length(path)-1
    plot(nodes(path(ii:ii+1),2),nodes(path(ii:ii+1),3),'r:','linewidth',4);
    gif
%     F = getframe;
%     writeVideo(v,F);
end

% % movie(F,1,1);

% close(v);
