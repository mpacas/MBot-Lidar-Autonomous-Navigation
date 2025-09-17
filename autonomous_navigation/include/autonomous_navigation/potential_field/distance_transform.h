#ifndef AUTONOMOUS_NAVIGATION_GRAPH_SEARCH_DISTANCE_TRANSFORM_H
#define AUTONOMOUS_NAVIGATION_GRAPH_SEARCH_DISTANCE_TRANSFORM_H

#include <vector>

#include <autonomous_navigation/utils/graph_utils.h>


void distanceTransform(GridGraph& graph);
void distanceTransformSlow(GridGraph& graph);
void distanceTransformManhattan(GridGraph& graph);
std::vector<float> distanceTransformEuclidean1D(std::vector<float>& init_dt);
void distanceTransformEuclidean2D(GridGraph& graph);

#endif  // AUTONOMOUS_NAVIGATION_GRAPH_SEARCH_DISTANCE_TRANSFORM_H
