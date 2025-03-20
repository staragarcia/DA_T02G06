#include "Graph.h"
#include "MutablePriorityQueue.h"
#include "PairHash.hpp"
#include <unordered_set>

template <class T>
void genericDijkstra(const Graph<T>& g, Vertex<T>* source, std::function<std::vector<Edge<T> *>(Vertex<T>*)> getEdges, std::function<Vertex<T>*(Edge<T>*)> getVertex, std::function<bool(Vertex<T>*)> returnCondition, std::function<bool(Vertex<T>*, Vertex<T>*, Edge<T>*)> relaxCondition, std::function<void(Vertex<T>*, Vertex<T>*, Edge<T>*)> relaxAction, std::function<void(Vertex<T>*)> actionOnExtractedVertex) {
    if (source == nullptr) {
        return;
    }
    source->setDist(0);
    MutablePriorityQueue<Vertex<T>> pq;
    std::vector<Vertex<T>*> visitedVertices = {source};
    pq.insert(source);
    while (!pq.empty()) {
        Vertex<T>* v = pq.extractMin();
        if (returnCondition(v)) {
            cleanUp(visitedVertices);
            return;
        }
        actionOnExtractedVertex(v);
        for (Edge<T>* e : getEdges(v)) {
            Vertex<T>* u = getVertex(e);
            if (relaxCondition(u, v, e)) {
                relaxAction(u, v, e);
                if (!u->isVisited()) {
                    pq.insert(u);
                    visitedVertices.push_back(u);
                    u->setVisited(true);
                } else {
                    pq.decreaseKey(u);
                }
            }
        }
    }
    cleanUp(visitedVertices);
}