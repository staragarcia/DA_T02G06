#pragma once

#include "../utils/RestrictedDijkstra.hpp"
#include "../utils/GetDrivingPath.hpp"
#include <list>
#include <vector>

template <class T> 
void IndependentRoutePlanning(Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, std::list<T>& bestPath, int& bestTime, std::list<T>& altPath, int& altTime) {
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