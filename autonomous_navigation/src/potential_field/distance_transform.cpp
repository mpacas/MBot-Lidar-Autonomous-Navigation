
#include <iostream>
#include <vector>
#include <cmath>

#include <autonomous_navigation/utils/math_helpers.h>
#include <autonomous_navigation/utils/graph_utils.h>

#include <autonomous_navigation/potential_field/distance_transform.h>

/**
 * These functions should calculate the distance transform on the given graph
 * and store it in graph.obstacle_distances.
 **/

void distanceTransform(GridGraph& graph)
{
    /**
     * This function is called by the web app server. You can control which
     * distance transform function is used by uncommenting the relav
     ent
     * function, and commenting out the other two.
     */

    //distanceTransformSlow(graph);
    distanceTransformManhattan(graph);
    //distanceTransformEuclidean2D(graph);
}


void distanceTransformSlow(GridGraph& graph)
{
    /**
     * TODO (P2): Perform a distance transform by finding the distance to the
     * nearest occupied cell for each unoccupied cell. Calculate the distance
     * to the nearest cell by looping through all the occupied cells in the
     * graph.
     *
     * Store the result in the vector graph.obstacle_distances.
     **/
    int N = graph.width * graph.height;
//Initialization
   std::cout << "Initialization" << "\n";
    for(int i = 0; i < N; i++) {
        // If a Cell is Occupied, set its distance to 0
        if(isIdxOccupied(i, graph)){
            graph.obstacle_distances[i] = 0;
        }
        else{ //Otherwise, set it to high
            graph.obstacle_distances[i] = HIGH;    
        }
    }
    
    for(int i = 0; i < N; i++) {
        //std::cout << "Current Cell: " << i << " "  << "Initialized Cell Value: " << graph.obstacle_distances[i] << "\n" ;
        Cell current = idxToCell(i, graph);
        if(! isIdxOccupied(i, graph)) {
           // std::cout << "This Logic Gate Works" << "\n";
            for(int j = 0; j < N; j++) {
           // std::cout << j << "\n";
                if(isIdxOccupied(j, graph)) {
               
                    Cell observed = idxToCell(j, graph);
                        float dist; 
                    dist = sqrt(pow(current.i - observed.i, 2) + pow(current.j - observed.j, 2));
                        if(dist < graph.obstacle_distances[i]) {
                            graph.obstacle_distances[i] = dist;
                        }
                }
            }
        } //std::cout << "Current Cell: " << i << " "  << "Distance Value: " << graph.obstacle_distances[i] << "\n" ;
    }
    std::cout << "Distance Transform Complete" << "\n "<< "Width: " << graph.width << "\n" << "Height: " << graph.height << "\n";
}


void distanceTransformManhattan(GridGraph& graph)
{
    /**
     * TODO (P2): Perform a distance transform using the Manhattan distance
     * transform algorithm over a 2D grid.
     *
     * Store the result in the vector graph.obstacle_distances.
     **/
    int N = graph.width * graph.height;
//Initialization
    for(int i = 0; i < N; i++) {
        if(isIdxOccupied(i, graph)){
            graph.obstacle_distances[i] = 0;
        }
        else
        graph.obstacle_distances[i] = HIGH;
    }

// Forward Pass
    //assigns  graph.obstacle_distances at i to the minimum of  graph.obstacle_distances[i] and the location to the left of  graph.obstacle_distances[i]'s value + 1
   for(int i = 1; i <= (N - 1); i++){
        //on the forward pass any that doesnt have a bottom neighbor uses this if statemnt
        if( i<graph.width){

            graph.obstacle_distances[i] = std::min(graph.obstacle_distances[i], graph.obstacle_distances[i-1]+1);
     
        }
        //if it doesnt have a left neighbor this will execute
        else if(i % graph.width == 0){

            graph.obstacle_distances[i] = std::min(graph.obstacle_distances[i], graph.obstacle_distances[i-graph.width]+1);

        }
        //if it has a bottom and left neighbor this will execute
        else{

             graph.obstacle_distances[i] = std::min(graph.obstacle_distances[i], graph.obstacle_distances[i-graph.width]+1);
             graph.obstacle_distances[i] = std::min(graph.obstacle_distances[i], graph.obstacle_distances[i-1]+1);

        }
      
    }
    
    // Backward Pass
    //assigns  graph.obstacle_distances at i to the minimum of  graph.obstacle_distances[i] and the location to the right of  graph.obstacle_distances[i]'s value + 1
    for(int i = (N-2); i >= 0; i--) {

        //if it doesnt have a top neighbor this will execute
       if(i >= (N-graph.width)){
         graph.obstacle_distances[i] = std::min( graph.obstacle_distances[i],  graph.obstacle_distances[i+1]+1);

      }
      //if it doesnt have a right neighbor this will execute
      else if(i % graph.width == 1){
         graph.obstacle_distances[i] = std::min( graph.obstacle_distances[i],  graph.obstacle_distances[i+graph.width]+1);
      }
      //if it has a top and right neighbor this will execute
      else{
         graph.obstacle_distances[i] = std::min( graph.obstacle_distances[i],  graph.obstacle_distances[i+graph.width]+1);
         graph.obstacle_distances[i] = std::min( graph.obstacle_distances[i],  graph.obstacle_distances[i+1]+1);
      }

    }
    
    
}

// TODO: Implement the 1D distance transform of a binary map.
std::vector<float> distanceTransformEuclidean1D(std::vector<float>& init_dt)
{
    std::vector<float> dt(init_dt.begin(), init_dt.end());

    /**
     * TODO (P2 - Advanced Extension): Perform a distance transform using the
     * Euclidean distance transform algorithm over a 1D vector using the initial
     * values provided in init_dt.
     *
     * Store the result in the vector dt.
     **/

    return dt;
}


void distanceTransformEuclidean2D(GridGraph& graph)
{
    /**
     * TODO (P2 - Advanced Extension): Perform a distance transform using the
     * Euclidean distance transform algorithm over a 2D grid. Use the
     * distanceTransformEuclidean1D() function.
     *
     * Store the result in the vector graph.obstacle_distances.
     **/

    // Use celltoidx and idxtocell for the logic checks on the passes
}
