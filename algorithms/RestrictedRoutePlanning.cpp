#include "../utils/RestrictedDijkstra.hpp"
#include "../utils/GetDrivingPath.hpp"
#include <list>

template <class T> 
std::list<T> RestrictedRoutePlanning(Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, std::unordered_set<T> avoid_nodes, std::unordered_set<std::pair<T,T>> avoid_edges, Vertex<T>* include_node) {
    RestrictedDijkstra(g, source, include_node, avoid_nodes, avoid_edges);
    RestrictedDijkstra(g, include_node, dest, avoid_nodes, avoid_edges);
    std::list<T> path={};
    getDrivingPath(dest, path);
    return path;
}