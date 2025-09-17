#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <autonomous_navigation/utils/math_helpers.h>
#include <autonomous_navigation/utils/graph_utils.h>

#define LOG_HEADER "[TESTS] "
#define TINY_MAP_SIZE 10


void printMap(std::vector<bool>& map)
{
    int idx = 0;
    for (int i = 0; i < TINY_MAP_SIZE; ++i)
    {
        std::cout << "\t";
        for (int j = 0; j < TINY_MAP_SIZE; ++j)
        {
            char cell = map[idx] ? 'x' : 'o';
            std::cout << cell << " ";
            idx++;
        }
        std::cout << std::endl;
    }
}


bool testMapIndexing(const GridGraph& graph)
{
    std::vector<bool> true_tiny_map = {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 1, 1, 0, 0, 0, 0, 0, 1,
        1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 0, 0, 0, 0, 0, 1, 0, 1,
        1, 0, 0, 0, 1, 0, 0, 1, 0, 1,
        1, 0, 0, 1, 0, 0, 0, 1, 0, 1,
        1, 0, 1, 0, 0, 0, 0, 1, 0, 1,
        1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };

    std::cout << "If your indexing function is correct, your map should look like this:" << std::endl;
    printMap(true_tiny_map);
    std::cout << std::endl;

    std::vector<bool> test_tiny_map(TINY_MAP_SIZE * TINY_MAP_SIZE, 0);
    bool idx_correct = true;
    bool cell_correct = true;

    int arr_idx = 0;
    for (int i = 0; i < graph.height; ++i)
    {
        for (int j = 0; j < graph.width; j++)
        {
            // We will print the map upside down so it matches the picture.
            int idx = cellToIdx(graph.height - i - 1, j, graph);
            if (idx == -1)
            {
                std::cout << "\tcellToIdx() not implemented." << std::endl;
                return false;
            }
            // Test if cellToIdx() returns the right element.
            test_tiny_map[arr_idx] = isIdxOccupied(idx, graph);
            idx_correct = idx_correct && test_tiny_map[arr_idx] == true_tiny_map[arr_idx];

            // Test if idxToCell() returns the right cell.
            Cell c = idxToCell(arr_idx, graph);
            if (c.i != i || c.j != j)
            {
                cell_correct = false;
                std::cout << "\tIdx: " << arr_idx << " Your cell: (" << c.i << ", " << c.j << ") ";
                std::cout << "Correct cell: (" << i << ", " << j << ")\n";
            }
            arr_idx++;
        }
    }
    std::cout << "Your map:\n";
    printMap(test_tiny_map);
    std::cout << "\n";

    if (!idx_correct)
    {
        std::cout << LOG_HEADER << "cellToIdx() function is incorrect.\n";
    }
    if (!cell_correct)
    {
        std::cout << LOG_HEADER << "idxToCell() function is incorrect.\n";
    }

    return idx_correct && cell_correct;
}


int main(int argc, char const *argv[])
{
    std::string map_file = "../data/tiny_map.map";

    GridGraph graph;
    if (!loadFromFile(map_file, graph))
    {
        std::cerr << LOG_HEADER << "Failed to load file: " << map_file << std::endl;
        return -1;
    }
    std::cout << LOG_HEADER << "Map loaded: " << map_file << std::endl;

    // Test map indexing.
    std::cout << LOG_HEADER << "---------------------------------" << std::endl;
    std::cout << LOG_HEADER << "Testing map indexing." << std::endl;
    bool map_correct = testMapIndexing(graph);
    std::cout << LOG_HEADER << "---------------------------------" << std::endl;
    std::cout << LOG_HEADER << "Map indexing test: ";
    if (map_correct)
    {
        std::cout << "PASSED" << std::endl;
    }
    else std::cout << "FAILED :(" << std::endl;

    /**
     * You may add any other tests you want here.
     **/

    return 0;
}
