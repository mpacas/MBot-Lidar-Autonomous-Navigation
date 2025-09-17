#include <iostream>
#include <cmath>
#include <queue>
#include <functional>

#include <autonomous_navigation/utils/math_helpers.h>
#include <autonomous_navigation/utils/graph_utils.h>

#include <autonomous_navigation/graph_search/graph_search.h>

/**
 * General graph search instructions:
 *
 * First, define the correct data type to keep track of your visited cells
 * and add the start cell to it. If you need to initialize any properties
 * of the start cell, do so here.
 *
 * Next, implement the graph search function. Save the result in the path
 * variable defined for you.
 *
 * To visualize a cell, use the function showVisitedCell() passed to the graph
 * search functions as follows:
 *      showVisitedCell(i, j);
 * where i and j are the column and row respectively of the cell you want to
 * visualize. Call this function when you visit a cell so that it is shown on
 * the web app.
 *
 * At the end of your main loop, you can add a call to sleepFor(SECS) to leave
 * time to visualize the cell being visited. This might help you with
 * debugging.
 *
 * The tracePath() function will return a path (which you should assign to
 * the path variable above) given the goal index, if you have kept track
 * of the parent of each node correctly and have implemented the
 * getParent() function. If you do not find a path, return an empty path
 * vector.
*/

std::vector<Cell> depthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal,
                                   std::function<void(int, int)> showVisitedCell)
{
    std::vector<Cell> path;  // The final path should be placed here.

    initGraph(graph);  // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /**
     * TODO (P3): Implement DFS.
     */

    return path;
}

std::vector<Cell> breadthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal,
                                     std::function<void(int, int)> showVisitedCell)
{
    std::vector<Cell> path;  // The final path should be placed here.

    initGraph(graph);  // Make sure all the node values are reset.

    // Get the index of the goal and the start point.
    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx = cellToIdx(goal.i, goal.j, graph);

    std::queue<int> visit_list; // Creates a queue named visit_list
    /**
     * TODO (P3): Implement BFS.
     */

    // Initialize Start Node, just to be very sure we're reinitializing everything about start node.
    
    
    
    graph.nodes[start_idx].parent = -1;
    graph.nodes[start_idx].distance = 0;
    graph.nodes[start_idx].queued = false;
    graph.nodes[start_idx].visited = false;

    std::cout << "Start Node Initilized" << "\n";

    visit_list.push(start_idx); // Create the visit list.

    while(!visit_list.empty() && goal_idx != visit_list.front()) 
    {
    
      std::cout <<"\n";
      
      int currentnode = visit_list.front(); //
      Cell currentcell = idxToCell(currentnode,graph); // Generates i and j values for currentnode, used later to find the distance between it and neighbors
       
        // Marks the cells being visited in grey
        showVisitedCell(currentcell.i,currentcell.j);
      
       std::cout << "Iteration Step Started" << "\n";
      
      //Dequeue Current Node to visit and mark it visited
      visit_list.pop();
      graph.nodes[currentnode].queued = false;
      graph.nodes[currentnode].visited = true;
      //Visit Neighbors and assign parents and distances
      
      std::vector<int> neighborlist = findNeighbors(currentnode,graph); // Gives vector of 
      
        //For Every Neighbor loop 8 times
      for(int i = 0; i < neighborlist.size(); i++)
        { 
           int localneighbor = i; //Tells us which of the 8 neighbors we are checking
           int globalneighbor = neighborlist[i]; // Tells us the global index of the neighbor we are checking
           Cell neighborcell = idxToCell(globalneighbor,graph);
          
          std::cout << "Neighbor being checked" << "\n" << "Local Neighbor Index: " << localneighbor << "\n" << "Global Neighbor Index: " << globalneighbor << "\n";
          
          // If the neighbor has not been queued nor visited nor is in collision add it to the visit list
            if(!graph.nodes[globalneighbor].visited && !graph.nodes[globalneighbor].queued && !checkCollisionFast(globalneighbor, graph))
            {
                visit_list.push(globalneighbor);
                graph.nodes[globalneighbor].queued = true;
            }
            
            // If the distance of neighbor is > than the distance of the current node + edgecost of current to neighbor, update dist of neighbor to current + edgecost
            
            float neighbordistance = sqrt(pow(currentcell.i - neighborcell.i, 2) + pow(currentcell.j - neighborcell.j, 2));
            
            if(graph.nodes[globalneighbor].distance > (neighbordistance + graph.nodes[currentnode].distance))
            {
                    
            

                //std::cout << "Pushed " << g.data[neighborlist[i]] << " to the queue" << "\n";
            
                //std::cout << "Visiting " << g.data[neighborlist[i]] << " distance from "<< g.data[currentnode] << " = " << getEdgeCosts(currentnode,g)[i] << "\n";
                //Update the neighbor's parent to be current node, and distance to be the distance of current node + edgecost
                graph.nodes[globalneighbor].parent = currentnode;
                graph.nodes[globalneighbor].distance = (neighbordistance + graph.nodes[currentnode].distance);

        
                //std::cout << "Distance of " << g.data[neighborlist[i]] << " updated to " << g.node[neighborlist[i]].dist << "\n";
                //std::cout << "Neighbor Node Distance: " << g.node[neighborlist[i]].dist << "\n";
                //std::cout << "Current Node Distance: " << g.node[currentnode].dist << "\n";
                
            }
 
        } 
        
    }
    
    path = tracePath(goal_idx, graph);
    return path;
    








}

std::vector<Cell> aStarSearch(GridGraph& graph, const Cell& start, const Cell& goal,
                              std::function<void(int, int)> showVisitedCell)
{
    std::vector<Cell> path;  // The final path should be placed here.

    initGraph(graph);  // Make sure all the node values are reset.
    //distance_transform(graph); // Run to ensure fast collision checking
    // Get the index of the goal and the start point.
    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx = cellToIdx(goal.i, goal.j, graph);
    
    // Create the visit list.
    std::queue<int> visit_list;

    // Initialize Start Node, just to be very sure we're reinitializing everything about start node.
    graph.nodes[start_idx].parent = -1;
    graph.nodes[start_idx].distance = 0;
    graph.nodes[start_idx].queued = false;
    graph.nodes[start_idx].visited = false;

    std::cout << "Start Node Initilized" << "\n";

    while(!visit_list.empty() && goal_idx != visit_list.front()) {

        std::cout <<"\n";
      int currentnode = visit_list.front();
      
       std::cout << "Iteration Step Started" << "\n" <<"Current Node: " << currentnode << "\n";
      //Dequeue Current Node to visit and mark it visited
      visit_list.pop();
      graph.nodes[currentnode].queued = false;
      graph.nodes[currentnode].visited = true;
      //Visit Neighbors and assign parents and distances
      
      //For Every Neighbor of our current node
      std::vector<int> neighborlist = findNeighbors(currentnode,graph);
      
        







































    }




    /**
     * TODO (P3): Implement A-star search.
     */

    return path;
}
