#pragma once

#include "../utils/Graph.h"
#include "../utils/GraphInitialization.cpp"
#include "../utils/PairHash.hpp"
#include "../utils/GetDrivingPath.hpp"
#include <unordered_set>
#include <list>

template <class T>
int walkingReverseDijsktra(const Graph<T>& g, Vertex<T>* source, const int maxWalkTime, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges, std::unordered_map<Vertex<T>*, double>& reacheableVertices, std::vector<Vertex<T>*>& visitedVertices) {
    if (source == nullptr) {
        return -1;
    }
    source->setDist(0);
    MutablePriorityQueue<Vertex<T>> pq;
    visitedVertices.push_back(source);
    pq.insert(source);
    while (!pq.empty()) {
        Vertex<T>* v = pq.extractMin();
        v->setProcessing(false);
        if (v->getDist() > maxWalkTime) {
            cleanUpVisitedAndDist(visitedVertices);
            if (reacheableVertices.size() == 0) {
                return -1;
            } else {
                return 0;
            }
        }
        if (v->getParking() && v != source) {
            reacheableVertices.insert({v, v->getDist()});
        }
        for (Edge<T>* e : v->getIncoming()) {
            Vertex<T>* u = e->getOrig();
            if (v->getDist() == std::numeric_limits<int>::max() || e->getWalkingTime() == std::numeric_limits<int>::max() || (u->isVisited()&&!u->isProcessing()) || avoid_nodes.find(u->getId()) != avoid_nodes.end() || avoid_edges.find({u->getId(), v->getId()}) != avoid_edges.end()) {
                continue;
            }
            if (u->getDist() > v->getDist() + e->getWalkingTime()) {
                u->setDist(v->getDist() + e->getWalkingTime());
                u->setWalkingPath(e);
                if (!u->isVisited()) {
                    pq.insert(u);
                    visitedVertices.push_back(u);
                    u->setVisited(true);
                    u->setProcessing(true);
                } else if (u->isProcessing()) {
                    pq.decreaseKey(u);
                }
            }
        }
    }
    cleanUpVisitedAndDist(visitedVertices);
    if (reacheableVertices.size() == 0) {
        return -1;
    } else {
        return 0;
    }
}

template <class T>
Vertex<T>* drivingDijkstra(const Graph<T>& g, Vertex<T>* source, const std::unordered_set<T>& avoidNodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoidEdges, std::unordered_map<Vertex<T>*, double>& reacheableWalkingVertices, int& walkingTime, int& drivingTime, std::vector<Vertex<T>*>& visitedVertices) {
   if (source == nullptr) {
       return nullptr;
   }
   Vertex<T>* parkingNode = nullptr;
   int parkingNodeCost = std::numeric_limits<int>::max();
   int numReacheableWalkingVertices = reacheableWalkingVertices.size();
   source->setDist(0);
   MutablePriorityQueue<Vertex<T>> pq;
   visitedVertices.push_back(source);
   pq.insert(source);
   while (!pq.empty()) {
      Vertex<T>* v = pq.extractMin();
      v->setProcessing(false);
      if (reacheableWalkingVertices.find(v) != reacheableWalkingVertices.end()) {
         numReacheableWalkingVertices--;
         auto it = reacheableWalkingVertices.find(v);
         if (v != source && it->second + v->getDist() < parkingNodeCost) {
            parkingNode = v;
            parkingNodeCost = it->second + v->getDist();
            walkingTime = it->second;
            drivingTime = v->getDist();
         }
         if (numReacheableWalkingVertices == 0) {
            cleanUpVisitedAndDist(visitedVertices);
            return parkingNode;
         }
      }
      for (Edge<T>* e : v->getAdj()) {
         Vertex<T>* u = e->getDest();
         if (v->getDist() == std::numeric_limits<int>::max() || e->getDrivingTime() == std::numeric_limits<int>::max() || (u->isVisited()&&!u->isProcessing()) || avoidNodes.find(u->getId()) != avoidNodes.end() || avoidEdges.find({v->getId(), u->getId()}) != avoidEdges.end()) {
           continue;
         }
         if (u->getDist() > v->getDist() + e->getDrivingTime()) {
           u->setDist(v->getDist() + e->getDrivingTime());
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
   return parkingNode;
}


template <class T> 
void getDrivingAndWalkingPath(Vertex<T>* parkingNode, std::list<T>& orderedIds) {
    if (parkingNode == nullptr) {
        orderedIds = {};
        return;
    }
    Edge<T>* aux_edge = parkingNode->getWalkingPath();
    while (aux_edge != nullptr) {
        orderedIds.push_back(aux_edge->getDest()->getId());
        aux_edge = aux_edge->getDest()->getWalkingPath();
    }
    getDrivingPath(parkingNode, orderedIds);
}

template <class T>
int calculateEnvironmentallyFriendlyPath(const Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, const int maxWalkTime, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges, std::list<T>& path, T& parkingNodeId, int& walkingTime, int& drivingTime) {
    std::unordered_map<Vertex<T>*, double> reacheableWalkingVertices = {};
    std::vector<Vertex<T>*> visitedVertices = {};
    int err = walkingReverseDijsktra(g, dest, maxWalkTime, avoid_nodes, avoid_edges, reacheableWalkingVertices, visitedVertices);
    if (err == -1) {
        path = {};
        return -1;
    }
    Vertex<T>* parkingNode = drivingDijkstra(g, source, avoid_nodes, avoid_edges, reacheableWalkingVertices, walkingTime, drivingTime, visitedVertices);
    if (parkingNode == nullptr) {
        path = {};
        return -1;
    }
    parkingNodeId=parkingNode->getId();
    std::list<T> orderedIds = {};
    getDrivingAndWalkingPath(parkingNode, orderedIds);
    cleanUpPaths(visitedVertices);
    path = orderedIds;
    return 0;
}