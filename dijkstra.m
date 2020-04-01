function [shortDist,trace] = dijkstra(nodes,segments,start_ID,end_ID)
%djikstra Find shortest path for a given graph
%   [dist,path] = dijkstra(nodes, segments, start_ID, end_ID) finds the
%   shortest distance for given inputs.
%
%   Note: All arguments are mandatory
%   
% Inputs:
%     NODES is a Nx3 with the format [ID X Y] where ID is each node's ID
%     and (X, Y) are coordinates for node
%     SEGMENTS is a Nx3 matrix with the format [ID N1 N2]where ID is unique
%     segment ID and N1 and N2 are nodeIDs
%     START_ID nodeID for starting node
%     END_ID nodeID for ending node
% 
% Outputs:
%     DIST is the shortest Euclidean distance for path from start node to
%     end node
%     PATH is a list of nodes containing the shortest route from start node
%     to end node
%
% Helper Functions:
%     UPDATE updates the value for a particular column and row ID in a
%     table
%     VALUE return the value corresponding to particular column and row ID
%     DISTANCE return euclidean distance between two nodes
%
%
% Author: VARUN AGGARWAL
% Date: 3/30/2020
% Github: https://github.com/Salazar-Prime/Dijkstra/


% create a distance table for minimum distance to a node
total_nodes = length(nodes);
dist = inf(total_nodes,1);
nodeID = nodes(:,1);
dist = table(nodeID, dist);

% create a path table to store the most adjacent parent node for
% minimum distance
path = table(nodeID, nodeID);
path.Properties.VariableNames = {'nodeID', 'parent'};
clear nodeID;

% initialize start node distance to zero
dist = update(dist, start_ID, 'dist', 0);

% find distances for all nodes
for i=1:total_nodes
    dist = sortrows(dist, 2);
    current_node = dist.nodeID(i);

    % neighbours of each node
    for neighbour=segments(segments(:,2)==current_node,3)'
        previous_dist = value(dist, neighbour, 'dist');
        current_dist = distance(nodes, current_node, neighbour) + value(dist, current_node, 'dist');
        % check if new path distance is less than previous distance
        if current_dist < previous_dist
            dist = update(dist, neighbour, 'dist', current_dist);
            path = update(path, neighbour, 'parent', current_node);
        end
    end
end

%% extract shortest path and its distance

% distance
shortDist = value(dist, end_ID, 'dist');

% backtrace path
node = end_ID;
trace = [node];
while node ~= start_ID
    node = value(path, node, 'parent');
    trace = [trace, node];    
end

end % end function

%% helper functions for table manipulation

% to update table value 
function table = update(table, ID, col, value)
    eval(strcat('table.',col,'(table.nodeID == ID) = value;'));
end

% to fetch data corresponding to a nodeID from table 
function dist = value(table, ID, col)
    eval(strcat('dist = table.',col,'(table.nodeID==ID);'))
end

% calculate euclidean distance 
function dist = distance(nodes, A, B)
    A = nodes(nodes(:,1)==A,2:3);
    B = nodes(nodes(:,1)==B,2:3);
    dist = norm(B-A);
end
