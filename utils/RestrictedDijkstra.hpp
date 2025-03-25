#pragma once

#include "../utils/Graph.h"
#include "../utils/PairHash.hpp"
#include <unordered_set>

/**
 * @brief Runs a Dijkstra algorithm on a graph, finding a path from the source to the dest node, while avoiding certain nodes and edges
 * 
 * @tparam T 
 * @param g graph where the path will be found
 * @param source Initial node of the path
 * @param dest Final node of the path
 * @param avoid_nodes nodes that the path can't go through
 * @param avoid_edges edges that the path can't go through
 */
template <class T>
int RestrictedDijkstra(const Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges, std::vector<Vertex<T>*>& visitedVertices) {
    int time;
    if (source == nullptr) {
        return 0;
    }
    source->setDist(0);
    MutablePriorityQueue<Vertex<T>> pq;
    visitedVertices.push_back(source);
    pq.insert(source);
    while (!pq.empty()) {
        Vertex<T>* v = pq.extractMin();
        if (v == dest) {
            time = dest->getDist();
            cleanUpVisitedAndDist(visitedVertices);
            return time;
        }
        for (Edge<T>* e : v->getAdj()) {
            Vertex<T>* u = e->getDest();
            if (avoid_nodes.find(u->getId()) != avoid_nodes.end() || avoid_edges.find({v->getId(), u->getId()}) != avoid_edges.end()) {
                continue;
            }
            if (u->getDist() > v->getDist() + e->getDrivingTime()) {
                u->setDist(v->getDist() + e->getDrivingTime());
                u->setPath(e);
                if (!u->isVisited()) {
                    pq.insert(u);
                    u->setVisited(true);
                    visitedVertices.push_back(u);
                } else {
                    pq.decreaseKey(u);
                }
            }
        }
    }
    time = dest->getDist();
    cleanUpVisitedAndDist(visitedVertices);
    return time;
}