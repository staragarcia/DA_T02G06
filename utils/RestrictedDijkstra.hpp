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
    if (source == nullptr) {
        return 0;
    }
    int time;
    source->setDist(0);
    MutablePriorityQueue<Vertex<T>> pq;
    visitedVertices.push_back(source);
    pq.insert(source);
    while (!pq.empty()) {
        Vertex<T>* v = pq.extractMin();
        v->setProcessing(false);
        if (v == dest) {
            time = dest->getDist();
            cleanUpVisitedAndDist(visitedVertices);
            return time;
        }
        for (Edge<T>* e : v->getAdj()) {
            Vertex<T>* u = e->getDest();
            if (v->getDist() == std::numeric_limits<int>::max() || e->getDrivingTime() == std::numeric_limits<int>::max() || (u->isVisited()&&!u->isProcessing())  || avoid_nodes.find(u->getId()) != avoid_nodes.end() || avoid_edges.find({v->getId(), u->getId()}) != avoid_edges.end()) {
                continue;
            }
            int cost = v->getDist() + e->getDrivingTime();
            if (u->getDist() > cost) {
                u->setDist(cost);
                u->setPath(e);
                if (!u->isVisited()) {
                    pq.insert(u);
                    u->setVisited(true);
                    u->setProcessing(true);
                    visitedVertices.push_back(u);
                } else if (u->isProcessing()) {
                    pq.decreaseKey(u);
                }
            }
        }
    }
    cleanUpVisitedAndDist(visitedVertices);
    return -1;
}