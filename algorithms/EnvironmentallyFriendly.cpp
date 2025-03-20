#include "../utils/Graph.h"
#include "../utils/GraphInitialization.cpp"
#include "../utils/PairHash.hpp"
#include "../utils/GetDrivingPath.hpp"
#include <unordered_set>
#include <list>

template <class T>
void walkingReverseDijsktra(const Graph<T>& g, Vertex<T>* source, const double maxWalkTime, std::unordered_map<Vertex<T>*, double>& reacheableVertices, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges) {
    if (source == nullptr) {
        return;
    }
    source->setDist(0);
    MutablePriorityQueue<Vertex<T>> pq;
    std::vector<Vertex<T>*> visitedVertices = {source};
    pq.insert(source);
    while (!pq.empty()) {
        Vertex<T>* v = pq.extractMin();
        if (v->getDist() > maxWalkTime) {
            cleanUp(visitedVertices);
            return;
        }
        if (v->getParking() && v != source) {
            reacheableVertices.insert({v, v->getDist()});
        }
        for (Edge<T>* e : v->getIncoming()) {
            Vertex<T>* u = e->getOrig();
            if (avoid_nodes.find(u->getId()) != avoid_nodes.end() || avoid_edges.find({u->getId(), v->getId()}) != avoid_edges.end()) {
                continue;
            }
            if (u->getDist() > v->getDist() + e->getWalkingTime()) {
                u->setDist(v->getDist() + e->getWalkingTime());
                u->setWalkingPath(e);
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

template <class T>
Vertex<T>* drivingDijkstra(const Graph<T>& g, Vertex<T>* source, std::unordered_map<Vertex<T>*, double>& reacheableWalkingVertices, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges) {
    if (source == nullptr) {
        return nullptr;
    }
    Vertex<T>* parkingNode = nullptr;
    double parkingNodeCost = std::numeric_limits<double>::max();
    int numReacheableWalkingVertices = reacheableWalkingVertices.size();
    source->setDist(0);
    MutablePriorityQueue<Vertex<T>> pq;
    std::vector<Vertex<T>*> visitedVertices = {source};
    pq.insert(source);
    while (!pq.empty()) {
        Vertex<T>* v = pq.extractMin();
        if (reacheableWalkingVertices.find(v) != reacheableWalkingVertices.end()) {
            numReacheableWalkingVertices--;
            auto it = reacheableWalkingVertices.find(v);
            if (v != source && it->second + v->getDist() < parkingNodeCost) {
                parkingNode = v;
                parkingNodeCost = it->second;
            }
            if (numReacheableWalkingVertices == 0) {
                cleanUp(visitedVertices);
                return parkingNode;
            }
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
    return parkingNode;
}


template <class T> 
void getDrivingAndWalkingPath(Vertex<T>* parkingNode, std::list<T>& orderedIds) {
    Edge<T>* aux_edge = parkingNode->getWalkingPath();
    while (aux_edge != nullptr) {
        orderedIds.push_back(aux_edge->getDest()->getId());
        aux_edge = aux_edge->getDest()->getWalkingPath();
    }
    getDrivingPath(parkingNode, orderedIds);
}

template <class T>
void calculatePath(const Graph<T>& g, const T sourceId, T destId, const double maxWalkTime, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges) {
    std::unordered_map<Vertex<T>*, double> reacheableWalkingVertices = {};
    walkingReverseDijsktra(g, g.findVertexById(destId), maxWalkTime, reacheableWalkingVertices, avoid_nodes, avoid_edges);
    Vertex<T>* parkingNode = drivingDijkstra(g, g.findVertexById(sourceId), reacheableWalkingVertices, avoid_nodes, avoid_edges);
    if (parkingNode == nullptr) {
        std::cout << "No path found" << std::endl;
        return;
    } else {
        std::cout << "Path found parking at: " << parkingNode->getId() << std::endl;
        std::list<T> orderedIds = {};
        getDrivingAndWalkingPath(parkingNode, orderedIds);
        for (T id : orderedIds) {
            std::cout << id << " ";
        }
        std::cout << std::endl;
    }
}

int main () {
    Graph<int> g;
    readParseLocations(g);
    readParseDistances(g);
    std::unordered_set<int> avoid_nodes = {13};
    std::unordered_set<std::pair<int, int>, pairHash> avoid_edges = {{2,7}};
    calculatePath(g, 1, 15, 40, avoid_nodes, avoid_edges);
    return 0;
}