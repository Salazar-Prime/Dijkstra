# Dijkstra
Dijkstra Implementation in MATLAB to find shortest path for a given graph

### dijkstra.m
   **Usage:**  
   ```
   [dist,path] = dijkstra(nodes, segments, start_ID, end_ID) 
   
   Note: All arguments are mandatory
   ```
   
 **Inputs:**
 
     NODES is a Nx3 with the format [ID X Y] where ID is each node's ID
     and (X, Y) are coordinates for node
     SEGMENTS is a Nx3 matrix with the format [ID N1 N2]where ID is unique
     segment ID and N1 and N2 are nodeIDs
     START_ID nodeID for starting node
     END_ID nodeID for ending node
 
 **Outputs:**
 
     DIST is the shortest Euclidean distance for path from start node to
     end node
     PATH is a list of nodes containing the shortest route from start node
     to end node

 **Helper Functions:**
 
     UPDATE updates the value for a particular column and row ID in a
     table
     VALUE return the value corresponding to particular column and row ID
     DISTANCE return euclidean distance between two nodes


 **Author: VARUN AGGARWAL  
 Date: 3/30/2020  
 Github: https://github.com/Salazar-Prime/Dijkstra/**
