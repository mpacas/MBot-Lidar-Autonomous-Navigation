#ifndef AUTONOMOUS_NAVIGATION_GRAPH_SEARCH_GRAPH_SEARCH_H
#define AUTONOMOUS_NAVIGATION_GRAPH_SEARCH_GRAPH_SEARCH_H

#include <vector>
#include <functional>


std::vector<Cell> depthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal,
                                   std::function<void(int, int)> showVisitedCell);
std::vector<Cell> breadthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal,
                                     std::function<void(int, int)> showVisitedCell);
std::vector<Cell> aStarSearch(GridGraph& graph, const Cell& start, const Cell& goal,
                              std::function<void(int, int)> showVisitedCell);


inline std::vector<Cell> depthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    return depthFirstSearch(graph, start, goal, [](int i, int j) {});
}

inline std::vector<Cell> breadthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    return breadthFirstSearch(graph, start, goal, [](int i, int j) {});
}

inline std::vector<Cell> aStarSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    return aStarSearch(graph, start, goal, [](int i, int j) {});
}

#endif  // AUTONOMOUS_NAVIGATION_GRAPH_SEARCH_GRAPH_SEARCH_H
