#pragma once

#include "../utils/RestrictedDijkstra.hpp"
#include "../utils/GetDrivingPath.hpp"
#include <list>

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