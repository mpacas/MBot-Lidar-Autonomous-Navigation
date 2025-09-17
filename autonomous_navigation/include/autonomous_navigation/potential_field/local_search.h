#ifndef AUTONOMOUS_NAVIGATION_POTENTIAL_FIELD_LOCAL_SEARCH_H
#define AUTONOMOUS_NAVIGATION_POTENTIAL_FIELD_LOCAL_SEARCH_H

#include <vector>

#include <autonomous_navigation/utils/graph_utils.h>

/**
 * Finds the index of the neighbor with the greatest potential drop from the
 * current cell.
 * @param  curr_idx  The current index.
 * @param  nbrs      The neighbors of the current index to check.
 * @param  field     The potential field.
 * @return  The index of the cell in nbrs with the greatest potential drop, or
 *          -1 if the current cell has lower potential than all its neighbors.
 **/
int findLowestNbr(int curr_idx, std::vector<int>& nbrs, const std::vector<float>& field);

/**
 * Calculates a vector in the robot frame pointing towards the direction of
 * greatest potential decrease.
 * @param  x      The current x position (meters).
 * @param  y      The current y position (meters).
 * @param  theta  The current angle (radians).
 * @param  graph  The graph.
 * @param  field  The potential field.
 * @param  depth  The depth to search for a lower neighbor (default: 5).
 * @return  A vector of length 3 where the first two elements are the x and y
            coordinate of the vector pointing in the direction of greatest
            decrease and the last element is the magnitude of potential decrease.
 **/
std::vector<float> localSearch(float x, float y, float theta,
                               GridGraph& graph, const std::vector<float>& field,
                               int depth = 5);

/**
 * Calculates the full path from the start to a local minimum in the given field.
 **/
std::vector<Cell> localSearchFull(const Cell& start, GridGraph& graph, const std::vector<float>& field);

#endif  // AUTONOMOUS_NAVIGATION_POTENTIAL_FIELD_LOCAL_SEARCH_H
