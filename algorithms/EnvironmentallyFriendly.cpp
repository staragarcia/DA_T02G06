#pragma once

#include "../utils/Graph.h"
#include "../utils/GraphInitialization.cpp"
#include "../utils/PairHash.hpp"
#include "../utils/GetDrivingPath.hpp"
#include <unordered_set>
#include <list>

template <class T>
void walkingReverseDijsktra(const Graph<T>& g, Vertex<T>* source, const double maxWalkTime, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges, std::unordered_map<Vertex<T>*, double>& reacheableVertices, std::vector<Vertex<T>*>& visitedVertices) {
    if (source == nullptr) {
        return;
    }
    source->setDist(0);
    MutablePriorityQueue<Vertex<T>> pq;
    visitedVertices.push_back(source);
    pq.insert(source);
    while (!pq.empty()) {
        Vertex<T>* v = pq.extractMin();
        if (v->getDist() > maxWalkTime) {
            cleanUpVisitedAndDist(visitedVertices);
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
    cleanUpVisitedAndDist(visitedVertices);
}

template <class T>
Vertex<T>* drivingDijkstra(const Graph<T>& g, Vertex<T>* source, const std::unordered_set<T>& avoidNodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoidEdges, std::unordered_map<Vertex<T>*, double>& reacheableWalkingVertices, int& totalTime, std::vector<Vertex<T>*>& visitedVertices) {
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
      if (reacheableWalkingVertices.find(v) != reacheableWalkingVertices.end()) {
         numReacheableWalkingVertices--;
         auto it = reacheableWalkingVertices.find(v);
         if (v != source && it->second + v->getDist() < parkingNodeCost) {
            parkingNode = v;
            parkingNodeCost = it->second;
         }
         if (numReacheableWalkingVertices == 0) {
            cleanUpVisitedAndDist(visitedVertices);
            totalTime = parkingNodeCost;
            return parkingNode;
         }
      }
      for (Edge<T>* e : v->getAdj()) {
         Vertex<T>* u = e->getDest();
         if (avoidNodes.find(u->getId()) != avoidNodes.end() || avoidEdges.find({v->getId(), u->getId()}) != avoidEdges.end()) {
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
   cleanUpVisitedAndDist(visitedVertices);
   totalTime=parkingNodeCost;
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
int calculateEnvironmentallyFriendlyPath(const Graph<T>& g, const T sourceId, T destId, const double maxWalkTime, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges, std::list<T>& path, T& parkingNodeId) {
    std::unordered_map<Vertex<T>*, double> reacheableWalkingVertices = {};
    std::vector<Vertex<T>*> visitedVertices = {};
    walkingReverseDijsktra(g, g.findVertexById(destId), maxWalkTime, avoid_nodes, avoid_edges, reacheableWalkingVertices, visitedVertices);
    int totalTime;
    Vertex<T>* parkingNode = drivingDijkstra(g, g.findVertexById(sourceId), avoid_nodes, avoid_edges, reacheableWalkingVertices, totalTime, visitedVertices);
    parkingNodeId=parkingNode->getId();
    std::list<T> orderedIds = {};
    getDrivingAndWalkingPath(parkingNode, orderedIds);
    g->cleanUpPaths(visitedVertices);
    path = orderedIds;
    return totalTime;
}

int main () {
    Graph<int> g;
    readParseLocations(g);
    readParseDistances(g);
    std::unordered_set<int> avoid_nodes = {13};
    std::unordered_set<std::pair<int, int>, pairHash> avoid_edges = {{2,7}};
    std::list<int> path = {};
    int parkingNodeId;
    int totalTime = calculateEnvironmentallyFriendlyPath(g, 1, 15, 40, avoid_nodes, avoid_edges, path, parkingNodeId);
    if (path.size() == 0) {
        std::cout << "No path Found" << std::endl;
    } else {
        std::cout << "Parking at: " << parkingNodeId << std::endl;
        std::cout << "Total Time: " << totalTime << std::endl;
        std::cout << "Path: ";
        for (int i : path) {
            std::cout << i << " ";
        }
        std::cout << std::endl;
    }
    return 0;
}