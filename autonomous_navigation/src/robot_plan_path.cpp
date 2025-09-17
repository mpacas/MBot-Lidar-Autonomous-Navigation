#include <iostream>
#include <cmath>
#include <string>

#include <autonomous_navigation/robot/robot.h>
#include <autonomous_navigation/utils/graph_utils.h>
#include <autonomous_navigation/utils/math_helpers.h>
#include <autonomous_navigation/graph_search/graph_search.h>
#include <autonomous_navigation/potential_field/distance_transform.h>


int main(int argc, char const *argv[])
{
    float goal_x = 0, goal_y = 0;

    if (argc < 2)
    {
        std::cerr << "Please provide the path to a map file as input.\n";
        return -1;
    }

    if (argc == 4)
    {
        goal_x = std::stof(argv[2]);
        goal_y = std::stof(argv[3]);
    }

    // Get the map path and load map.
    std::string map_file = argv[1];
    GridGraph graph;
    loadFromFile(map_file, graph);
    // TODO: Call your distance transform function if using checkCollisionFast().
    distanceTransform(graph);
    // Convert goal position to cell.
    Cell goal = posToCell(goal_x, goal_y, graph);

    // Listen for pose and wait until pose has been received.
    float x = 0, y = 0, theta = 0;
    initPoseListener();
    while (!hasPose())
    {
        handle();
    }

    // Get the current pose and convert it to a cell.
    getPose(x, y, theta);
    Cell start = posToCell(x, y, graph);

    std::vector<Cell> path;
    // TODO: Call graph search function and put the result in path.
    path = breadthFirstSearch(graph,start,goal);
    // Send path to motion controller.
    drivePath(path, graph);

    return 0;
}
