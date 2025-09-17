#include <vector>
#include <cmath>
#include <iostream>

#include <autonomous_navigation/utils/graph_utils.h>
#include <autonomous_navigation/potential_field/local_search.h>


int findLowestNbr(int curr_idx, std::vector<int>& nbrs, const std::vector<float>& field)
{
    int grad_idx = 0;
    float grad = field[curr_idx] - field[nbrs[grad_idx]];
    for (int n = 1; n < nbrs.size(); n++)
    {
        float nbr_grad = field[curr_idx] - field[nbrs[n]];
        if (nbr_grad > grad)
        {
            grad_idx = n;
            grad = nbr_grad;
        }
    }

    // If none of the neighbors have lower potential than the current cell, we're done!
    if (grad <= 0)
    {
        return -1;
    }

    return grad_idx;
}

std::vector<float> localSearch(float x, float y, float theta,
                               GridGraph& graph, const std::vector<float>& field,
                               int depth)
{
    std::vector<float> v_grad(3, 0);   // vx, vy, |grad|
    // Get the current cell.
    Cell c = posToCell(x, y, graph);
    int curr_idx = cellToIdx(c.i, c.j, graph);
    int next_idx = curr_idx;  // This will be the next index to explore.

    // Depth must be at least 1.
    depth = std::max(1, depth);
    // Search at a given depth for the next lowest potential.
    for (int i = 0; i < depth; i++)
    {
        auto nbrs = findNeighbors(next_idx, graph);
        int grad_idx = findLowestNbr(next_idx, nbrs, field);
        // If the next indx is the lowest, stop searching.
        if (grad_idx < 0)  break;

        next_idx = nbrs[grad_idx];
    }

    // If The current cell is a local optimum, we're done!
    if (next_idx == curr_idx)
    {
        return v_grad;
    }

    // Save the gradient magnitude.
    float grad = field[curr_idx] - field[next_idx];
    v_grad[2] = grad;

    // Get the x and y coordinates of the vector in the map frame.
    auto cn = idxToCell(next_idx, graph);
    auto nbr_pos = cellToPos(cn.i, cn.j, graph);
    float vx_map = nbr_pos[0] - x;
    float vy_map = nbr_pos[1] - y;

    // Normalize the velocity vector.
    float norm = sqrt(vx_map * vx_map + vy_map * vy_map);
    vx_map /= norm;
    vy_map /= norm;

    // Move the vector to the robot's coordinate frame.
    v_grad[0] = vx_map * cos(theta) + vy_map * sin(theta);
    v_grad[1] = -vx_map * sin(theta) + vy_map * cos(theta);

    return v_grad;
}


std::vector<Cell> localSearchFull(const Cell& start, GridGraph& graph, const std::vector<float>& field)
{
    std::vector<Cell> path;
    path.push_back(start);
    int curr_idx = cellToIdx(start.i, start.j, graph);

    while(true)
    {
        auto nbrs = findNeighbors(curr_idx, graph);
        int grad_idx = findLowestNbr(curr_idx, nbrs, field);

        // If all the neighbors have higher cost than the current value, we've reached a local minimum.
        if (grad_idx < 0)
        {
            return path;
        }

        curr_idx = nbrs[grad_idx];
        path.push_back(idxToCell(curr_idx, graph));
    }
}
