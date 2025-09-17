#ifndef AUTONOMOUS_NAVIGATION_GRAPH_SEARCH_POTENTIAL_FIELD_H
#define AUTONOMOUS_NAVIGATION_GRAPH_SEARCH_POTENTIAL_FIELD_H

#include <vector>

#include <autonomous_navigation/utils/graph_utils.h>

std::vector<float> createPotentialField(GridGraph& graph, const Cell& goal);
std::vector<float> createAttractiveField(GridGraph& graph, const Cell& goal);
std::vector<float> createRepulsiveField(GridGraph& graph);

#endif  // AUTONOMOUS_NAVIGATION_GRAPH_SEARCH_POTENTIAL_FIELD_H
