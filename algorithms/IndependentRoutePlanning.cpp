#include "../utils/RestrictedDijkstra.hpp"
#include "../utils/GetDrivingPath.hpp"
#include <list>

template <class T> 
std::list<T> IndependentRoutePlanning(Graph<T>& g, Vertex<T>* source, Vertex<T>* dest) {
    RestrictedDijkstra(g, source, dest, {}, {{}});
    std::list<T> path={};
    getDrivingPath(dest, path);
    return path;
}