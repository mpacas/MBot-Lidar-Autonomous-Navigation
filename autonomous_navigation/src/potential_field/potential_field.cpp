#include <iostream>
#include <cmath>

#include <autonomous_navigation/utils/math_helpers.h>
#include <autonomous_navigation/utils/graph_utils.h>
#include <autonomous_navigation/potential_field/distance_transform.h>   
#include <autonomous_navigation/potential_field/potential_field.h>


std::vector<float> createPotentialField(GridGraph& graph, const Cell& goal)
{
    std::vector<float> potential_field(graph.width * graph.height, 0);
    std::vector<float> attractive_field(graph.width * graph.height, 0);
    std::vector<float> repul_field(graph.width * graph.height, 0);


    /**
     * TODO (P2): Using the graph and the given goal, create a potential field
     * which is HIGH in areas the robot should avoid and LOW where the robot
     * wants to go.
     *
     * Store the result in the vector potential_field, which should be indexed
     * the same way as the graph cell data.
     *
     * HINT: The potential field should be a combination of an attractive field
     * given by createAttractiveField() and a repulsive field created by
     * createRepulsiveField().
     *
     * HINT: Start by using only an attractive field and build from there!
     **/
    int N = graph.width*graph.height;
     attractive_field=createAttractiveField(graph,goal);
     repul_field = createRepulsiveField(graph);
    int i=0;
    while( i<N)
    {
    
        potential_field[i] = attractive_field[i] + repul_field[i];
        i++;

    }

    return potential_field;
}


std::vector<float> createAttractiveField(GridGraph& graph, const Cell& goal)
{
    std::vector<float> attractive_field(graph.width * graph.height, HIGH);

    /**
     * TODO (P2): Using the graph and the given goal, create an attractive field
     * which pulls the robot towards the goal. It should be HIGH when far away
     * from the goal, and LOW when close to the goal.
     *
     * Store the result in the vector attractive_field, which should be indexed
     * the same way as the graph cell data.
     goal.i goal.j**/
     

     int index;
     float rows,columns,distance;
     float max_distance=0;
     for(int i=0;i<graph.height;i++){
        for(int j=0;j<graph.width;j++){
           index=cellToIdx(i,j,graph); 
           rows=abs(goal.i-i);
           columns=abs(goal.j-j);
           distance=sqrt((pow(rows,2))+(pow(columns,2)));
           if(distance>max_distance){
               max_distance=distance;
           }

        }
    }
    for(int i=0;i<graph.height;i++){
        for(int j=0;j<graph.width;j++){
           index=cellToIdx(i,j,graph); 
           rows=abs(goal.i-i);
           columns=abs(goal.j-j);
           distance=sqrt((pow(rows,2))+(pow(columns,2)));
           attractive_field[index]=distance/max_distance;
        }
    }



    return attractive_field;
}


std::vector<float> createRepulsiveField(GridGraph& graph)
{
    std::vector<float> repulsive_field(graph.width * graph.height, 0);

    /**
     * TODO (P2): Using the distance transform stored in graph.obstacle_distances,
     * create a repulsive field which pushes the robot away from obstacles. It
     * should be HIGH when close to obstacles, and LOW when far from obstacles.
     *
     * Store the result in the vector repulsive_field, which should be indexed
     * the same way as the graph cell data.
     **/
    int index;
     
    float C=.5;
    for(int i=0;i<graph.height;i++){
        for(int j=0;j<graph.width;j++){
            index=cellToIdx(i,j,graph);
            
            repulsive_field[index]=exp(-C*graph.obstacle_distances[index]);
        }
        
    }


    return repulsive_field;
}
