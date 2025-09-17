
#include <iostream>
#include <cmath>
#include <string>
#include <signal.h>

#include <autonomous_navigation/robot/robot.h>
#include <autonomous_navigation/utils/graph_utils.h>
#include <autonomous_navigation/utils/math_helpers.h>
#include <autonomous_navigation/potential_field/potential_field.h>
#include <autonomous_navigation/potential_field/local_search.h>
#include <autonomous_navigation/potential_field/distance_transform.h>

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, char const *argv[])
{
    float goal_x = 0, goal_y = 0;

    // Get the map file from user input.
    if (argc < 2)
    {
        std::cerr << "Please provide the path to a map file as input.\n";
        return -1;
    }

    // If provided, get the goal location from user input.
    if (argc == 4)
    {
        goal_x = std::stof(argv[2]);
        goal_y = std::stof(argv[3]);
    }

    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    // Load the graph.
    std::string map_file = argv[1];
    GridGraph graph;
    loadFromFile(map_file, graph);
    distanceTransform(graph);

    // Convert the goal to a cell.
    Cell goal = posToCell(goal_x, goal_y, graph);

    std::vector<float> field;
    /**
     * TODO (P2): Call your function to create a potential field, and store the
     * result in field.
     **/
    field = createPotentialField(graph, goal);

    // Initialize the pose listener. The robot's state will be stored in (x, y, theta).
    float x = 0, y = 0, theta = 0;
    initPoseListener();

    /**
     * TODO (P2): Define any variables you need.
     **/
    float vel = 100;
    float maxVel=.5;
    float minVel = .25;
    float gradTune = .0035;
    while (true)
    {
        // Check for a new pose.
        handle();
        getPose(x, y, theta);

        // Do local search on the graph from the current position.
        std::vector<float> v_grad = localSearch(x, y, theta, graph, field);

        /**
         * TODO (P2): Use the vector pointing in the direction of steepest
         * descent of the potential field to drive the robot.
         *
         * v_grad contains 3 elements: {vx, vy, grad}. vx and vy form a vector
         * with magnitude 1 that point in the direction of steepest decrease
         * starting from the robot's current position. grad is the maginutude of
         * the decrease in potential.
         **/
        if(v_grad[2]>gradTune){

            std::cout << "grad " << v_grad[2]<<"   ";
            std::cout << "Current Speed " << vel*v_grad[2]*v_grad[0]<<"\n ";

            if(abs(vel*v_grad[2]*v_grad[0])>maxVel || abs(vel*v_grad[2]*v_grad[1])>maxVel){
                drive(maxVel*v_grad[0],maxVel*v_grad[1],0);
            }
            else if (abs(vel*v_grad[2]*v_grad[0])<minVel || abs(vel*v_grad[2]*v_grad[1])<minVel){

                drive(minVel*v_grad[0],minVel*v_grad[1],0);
            }
        
            else{

                drive(vel*v_grad[2]*v_grad[0],vel*v_grad[2]*v_grad[1],0); //includes multiplying speed by grad

            }
        }
        else{
            drive(0,0,0);
            break;
        }
        
    
        if (ctrl_c_pressed) break;
    }

    drive(0, 0, 0);
    return 0;
}
