#pragma once

#include "../utils/RestrictedDijkstra.hpp"
#include "../utils/GetDrivingPath.hpp"
#include <list>

/**
 * @brief This function computes the shortest path between the source and destination vertices, considering:
 * - Nodes to avoid.
 * - Edges to avoid.
 * - An optional intermediate node that must be included in the path.
 * The time complexity of the function is O((V + E) log V) for each call to Restricted Dijkstra, 
 * where V is the number of vertices and E is the number of edges in the graph.
 * If the include_node is specified, Restricted Dijkstra is called twice, resulting in an overall complexity of O(2 * (V + E) log V), 
 * which simplifies to O((V + E) log V).
 * 
 * @tparam T The type of the vertex identifiers (e.g., int, string).
 * @param g The graph where the path is calculated.
 * @param source Pointer to the source vertex.
 * @param dest Pointer to the destination vertex.
 * @param avoid_nodes A set of nodes that must be avoided in the path.
 * @param avoid_edges A set of edges that must be avoided in the path.
 * @param include_node Pointer to an intermediate node that must be included in the path (can be nullptr if not required).
 * @param path Reference to a list where the resulting path will be stored.
 * 
 * @return int The total time of the path (sum of weights), or -1 if no valid path is found.
 */
template <class T> 
int RestrictedRoutePlanning(Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, std::unordered_set<T> avoid_nodes, std::unordered_set<std::pair<T,T>, pairHash> avoid_edges, Vertex<T>* include_node, std::list<T>& path) {
    std::vector<Vertex<T>*> visitedVertices = {};
    int time;
    if (include_node != nullptr) {
        time = RestrictedDijkstra(g, include_node, dest, avoid_nodes, avoid_edges, visitedVertices);
        if (time == -1) {
            path.clear();
            return -1;
        }
        getDrivingPath(dest, path);
        path.pop_front();
        cleanUpPaths(visitedVertices);
        int time2 = RestrictedDijkstra(g, source, include_node, avoid_nodes, avoid_edges, visitedVertices);
        if (time2 == -1) {
            return -1;
        }
        time += time2;
        getDrivingPath(include_node, path);
        cleanUpPaths(visitedVertices);
    } else {
        time = RestrictedDijkstra(g, source, dest, avoid_nodes, avoid_edges, visitedVertices);
        if (time == -1) {
            return -1;
        }
        getDrivingPath(dest, path);
        cleanUpPaths(visitedVertices);
    }
    return time;
}