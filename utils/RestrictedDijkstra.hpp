#pragma once

#include "../utils/Graph.h"
#include "../utils/PairHash.hpp"
#include <unordered_set>

template <class T>
void RestrictedDijkstra(const Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges) {
    if (source == nullptr) {
        return;
    }
    source->setDist(0);
    MutablePriorityQueue<Vertex<T>> pq;
    std::vector<Vertex<T>*> visitedVertices = {source};
    pq.insert(source);
    while (!pq.empty()) {
        Vertex<T>* v = pq.extractMin();
        if (v == dest) {
            cleanUp(visitedVertices);
            return;
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
    cleanUp(visitedVertices);
}