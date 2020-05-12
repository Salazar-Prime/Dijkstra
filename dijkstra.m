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
%     DISTANCE return euclidean distance between two nodes
%
%
% Author: VARUN AGGARWAL
% Date: 3/30/2020
% Github: https://github.com/Salazar-Prime/Dijkstra/


% create a distance table for minimum distance to a node
total_nodes = length(nodes);
dist = inf(total_nodes,1);

% create a path array to store the most adjacent parent node for
% minimum distance
path = nodes(:,1);

% keeping track of visited nodes
visited = zeros(total_nodes,1);

% initialize start node distance to zero
dist(start_ID) = 0;

% find distances until end_ID is visited
while visited(end_ID) == 0
    % finding minimum distance out of not visited nodes
    current_node = min(setdiff( find(dist == min(dist(~visited))) , find(visited) ));
    visited(current_node) = 1; % mark current node as visited

    % neighbours of each node
    for neighbour=segments(segments(:,2)==current_node,3)'
        
        if visited(neighbour) ~= 1 % skip neighbours which have been visited already
            
            previous_dist = dist(neighbour);
            current_dist = distance(nodes, current_node, neighbour) + dist(current_node);
            
            % check if new path distance is less than previous distance
            if current_dist < previous_dist
                dist(neighbour) = current_dist;
                path(neighbour) = current_node;
            end
            
        end % end if
        
    end % end for
end % end while

%% extract shortest path and its distance

% distance
shortDist = dist(end_ID);
% backtrace path
node = end_ID;
trace = [node];
while node ~= start_ID
    node = path(node);
    trace = [trace, node];    
end

trace = flip(trace);

end % end function

%% helper functions

% calculate euclidean distance 
function dist = distance(nodes, A, B)
    A = nodes(nodes(:,1)==A,2:3);
    B = nodes(nodes(:,1)==B,2:3);
    dist = norm(B-A);
end
