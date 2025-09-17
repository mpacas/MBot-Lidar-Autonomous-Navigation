#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <autonomous_navigation/utils/graph_utils.h>


bool isLoaded(const GridGraph& graph)
{
    bool correct_size = graph.cell_odds.size() == graph.width * graph.height;
    bool positive_size = graph.width > 0 && graph.height > 0;
    bool positive_m_per_cell = graph.meters_per_cell > 0;
    return correct_size && positive_size && positive_m_per_cell;
}


bool loadFromFile(const std::string& file_path, GridGraph& graph)
{
    std::ifstream in(file_path);
    if (!in.is_open())
    {
        std::cerr << "ERROR: loadFromFile: Failed to load from " << file_path << std::endl;
        return false;
    }

    // Read header
    in >> graph.origin_x >> graph.origin_y >> graph.width >> graph.height >> graph.meters_per_cell;

    // Check sanity of values
    if (graph.width < 0 || graph.height < 0 || graph.meters_per_cell < 0.0f)
    {
        return false;
    }

    graph.collision_radius = ROBOT_RADIUS + graph.meters_per_cell;

    // Reset odds vector.
    int num_cells = graph.width * graph.height;
    graph.cell_odds.clear();
    graph.cell_odds.resize(num_cells);

    graph.obstacle_distances = std::vector<float>(graph.width * graph.height, 0);

    // Read in each cell value
    int odds;  // read in as an int so it doesn't convert the number to the corresponding ASCII code
    for (int idx = 0; idx < num_cells; ++idx)
    {
        in >> odds;
        graph.cell_odds[idx] = odds;
    }

    // Reset the nodes in the graph.
    initGraph(graph);

    return true;
};


void initGraph(GridGraph& graph)
{
    /**
     * TODO (P3): Initialize your graph nodes.
     *
     * When this funtion is called, the graph will have loaded the members
     * which store the properties of the graph, like width, height, and cell
     * odds values. You should use this information to initialize your added
     * values, like the distances and the nodes.
     */

    for(int i = 0; i < (graph.width * graph.height); i ++){
      // Populates vector graph.nodes with CellNodes that have initialized data in 
      CellNode targetnode;
      targetnode.parent = -1;
      targetnode.distance = HIGH;
      targetnode.visited = false;
      targetnode.queued = false;
      
      graph.nodes.push_back(targetnode);

    }
    std::cout << "Graph Initialization Complete" << "\n";
}


int cellToIdx(int i, int j, const GridGraph& graph)
{
    /**
     * TODO (P2): Return the index where the given cell is stored.
     */

    int idx = (j + (i * graph.width));

    return idx;
}


Cell idxToCell(int idx, const GridGraph& graph)
{
    Cell c;
    /**
     * TODO (P2): Set the row and column for the cell corresponding to idx.
     */
     c.i = (idx / graph.width);
     c.j = idx - (c.i*graph.width);

    return c;
}


Cell posToCell(float x, float y, const GridGraph& graph)
{
    int j = static_cast<int>(floor((x - graph.origin_x) / graph.meters_per_cell));
    int i = static_cast<int>(floor((y - graph.origin_y) / graph.meters_per_cell));

    Cell c;
    c.i = i;
    c.j = j;
    return c;
}


std::vector<float> cellToPos(int i, int j, const GridGraph& graph)
{
    float x = (j + 0.5) * graph.meters_per_cell + graph.origin_x;
    float y = (i + 0.5) * graph.meters_per_cell + graph.origin_y;

    return std::vector<float>({x, y});
}


bool isCellInBounds(int i, int j, const GridGraph& graph)
{
    return i >= 0 && j >= 0 && i < graph.height && j < graph.width;
}


bool isIdxOccupied(int idx, const GridGraph& graph)
{
    return graph.cell_odds[idx] >= graph.threshold;
}


bool isCellOccupied(int i, int j, const GridGraph& graph)
{
    return isIdxOccupied(cellToIdx(i, j, graph), graph);
}


std::vector<int> findNeighbors(int idx, const GridGraph& graph)
{
    std::vector<int> neighbors;
    Cell c = idxToCell(idx, graph);

    for (int i = c.i - 1; i <= c.i + 1; ++i)
    {
        for (int j = c.j - 1; j <= c.j + 1; ++j)
        {
            if (i == c.i && j == c.j) continue;  // Skip given cell.

            if (isCellInBounds(i, j, graph))
            {
                neighbors.push_back(cellToIdx(i, j, graph));
            }
        }
    }
    return neighbors;
}


bool checkCollision(int idx, const GridGraph& graph)
{
    // Check if this cell is in collision.
    if (isIdxOccupied(idx, graph))
    {
        return true;
    }

    // Use arc length of approximately 1 cell width.
    //   arc_length = theta * radius
    double dtheta = graph.meters_per_cell / graph.collision_radius;
    double theta = 0;
    auto c = idxToCell(idx, graph);
    auto state = cellToPos(c.i, c.j, graph);
    while (theta < 2 * M_PI)
    {
        double x = state[0] + graph.collision_radius * cos(theta);
        double y = state[1] + graph.collision_radius * sin(theta);
        Cell c = posToCell(x, y, graph);

        if (!isCellInBounds(c.i, c.j, graph))
        {
            return true;
        }

        if (isCellOccupied(c.i, c.j, graph))
        {
            return true;
        }

        theta += dtheta;
    }

    return false;
}

bool checkCollisionFast(int idx, const GridGraph& graph)
{
    return graph.obstacle_distances[idx] * graph.meters_per_cell <= graph.collision_radius;
}

int getParent(int idx, const GridGraph& graph)
{
    /**
     * TODO (P3): Return the parent of the node at idx.
     */

    return graph.nodes[idx].parent;
}


float getScore(int idx, const GridGraph& graph)
{
    /**
     * TODO (P3): Return the score of the node at idx.
     */
    return HIGH;
}


int findLowestScore(const std::vector<int>& node_list, const GridGraph& graph)
{
    int min_idx = 0;

    // Find the cell with the lowest score.
    for (int i = 1; i < node_list.size(); ++i)
    {
        if (getScore(node_list[i], graph) < getScore(node_list[min_idx], graph))
        {
            min_idx = i;
        }
    }

    // Return index of the lowest score cell.
    return min_idx;
}


std::vector<Cell> tracePath(int goal, const GridGraph& graph)
{
    std::vector<Cell> path;
    int current = goal;
    do
    {
        path.push_back(idxToCell(current, graph));
        current = getParent(current, graph);
    } while (current >= 0);  // A cell with no parent has parent -1.

    // Since we built the path backwards, we need to reverse it.
    std::reverse(path.begin(), path.end());
    return path;
}
