#pragma once

#include "../utils/Graph.h"
#include "../utils/PairHash.hpp"
#include <unordered_set>

template <class T>
int RestrictedDijkstra(const Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges) {
    int a;
    if (source == nullptr) {
        return 0;
    }
    source->setDist(0);
    MutablePriorityQueue<Vertex<T>> pq;
    std::vector<Vertex<T>*> visitedVertices = {source};
    pq.insert(source);
    while (!pq.empty()) {
        Vertex<T>* v = pq.extractMin();
        if (v == dest) {
            a = dest->getDist();
            cleanUp(visitedVertices);
            return a;
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
    a = dest->getDist();
    cleanUp(visitedVertices);
    return a;
}