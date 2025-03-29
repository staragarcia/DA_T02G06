#pragma once

#include "../utils/RestrictedDijkstra.hpp"
#include "../utils/GetDrivingPath.hpp"
#include <list>
#include <vector>

/**
 * @brief Calculates the shortest path and an alternative path between two vertices in a graph. The time complexity of the function is O((V + E) log V) for each call to Restricted Dijkstra, 
 *  where V is the number of vertices and E is the number of edges in the graph. Since Restricted Dijkstra is called twice (once for the best path and once for the alternative path), the overall complexity is O(2 * (V + E) log V), which simplifies to O((V + E) log V).
 * 
 * @tparam T The type of the vertex identifiers (e.g., int, string).
 * @param g The graph where the paths are calculated.
 * @param source Pointer to the source vertex.
 * @param dest Pointer to the destination vertex.
 * @param bestPath Reference to a list where the best path (shortest path) will be stored.
 * @param bestTime Reference to an integer where the time of the best path will be stored.
 * @param altPath Reference to a list where the alternative path will be stored.
 * @param altTime Reference to an integer where the time of the alternative path will be stored.
 */
template <class T> 
void IndependentRoutePlanning(Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, std::list<T>& bestPath, int& bestTime, std::list<T>& altPath, int& altTime) {
    if (source==nullptr || dest==nullptr) {
        std::cout << "Source or destination can't be null!\n";
        return;
    }
    std::vector<Vertex<T>*> visitedVertices = {};
    std::unordered_set<int> usedNodes;
    bestTime = RestrictedDijkstra(g, source, dest, {}, {{}}, visitedVertices);
    getDrivingPath(dest, bestPath);
    cleanUpPaths(visitedVertices);
    T sourceId = source->getId();
    T destinationId = dest->getId();

    for (T nodeId : bestPath) {
        if (nodeId != sourceId && nodeId != destinationId) {
            usedNodes.insert(nodeId);
        }
    }
    visitedVertices.clear();
    altTime = RestrictedDijkstra(g, source, dest, usedNodes, {{}}, visitedVertices);
    getDrivingPath(dest, altPath);
    cleanUpPaths(visitedVertices);
}