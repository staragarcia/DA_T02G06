#pragma once

#include "../utils/Graph.h"
#include "../utils/GraphInitialization.cpp"
#include "../utils/PairHash.hpp"
#include "../utils/GetDrivingPath.hpp"
#include <unordered_set>
#include <list>

/**
 * @brief Performs a Dijkstra's algorithm from source backwards (in the incoming edges of each node) using as distance the walking distance.
 * Total complexity is O((V+E)logV) where V is the number of vertices and E is the number of edges in the graph.
 * 
 * @tparam T 
 * @param g graph where the Dijkstra will be performed.
 * @param source node from which the Dijkstra will be calculated. Should be the destination of the path.
 * @param maxWalkTime maximum walking time allowed.
 * @param avoid_nodes nodes that the path can't go through.
 * @param avoid_edges edges that the path can't go through.
 * @param reacheableVertices map that relates ids of vertices and the time needed to reach source from them walking. Only nodes which have parking spots and that are at a walking distance smaller than maxWalkTime are included.
 * @param visitedVertices should be initialized as an empty vector. It will contain all the vertices visited during the algorithm.
 * @return int returns 0 as success, -1 if no reacheable vertices were found.
 */
template <class T>
int walkingReverseDijsktra(const Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, const int maxWalkTime, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges, std::unordered_map<Vertex<T>*, double>& reacheableVertices, std::vector<Vertex<T>*>& visitedVertices) {
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
        if (v->getParking() && v != source && v!=dest) {
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

/**
 * @brief After a walkingReverseDijsktra is performed, this function performs a driving Dijkstra starting at source. This also keeps track of the total time needed to traverse a path that parks at each of the nodes in reacheableWalkingNodes, and the node that minimizes that total time will be considered the parking node. The total
 * complexity is O((V+E)logV) where V is the number of vertices and E is the number of edges in the graph.
 * 
 * @tparam T 
 * @param g graph
 * @param source the node from which the Dijkstra will be performed
 * @param avoidNodes nodes that the path can't go through
 * @param avoidEdges edges that the path can't go through
 * @param reacheableWalkingVertices map between the ids of nodes that are candidates to be parking nodes and the time needed to walk to them from source
 * @param walkingTime used to return the walking time of the best path
 * @param drivingTime used to return the driving time of the best path
 * @param visitedVertices vertices that the Dijsktra visited
 * @return Vertex<T>* 
 */
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

/**
 * @brief After driving and walking Dijkstra are performed, this function fetches the best path in O(V) time complexity where V is the number of vertices in the graph.
 * 
 * @tparam T 
 * @param parkingNode node where the user parks and starts walking
 * @param orderedIds used to return the ordered ids of nodes in the path
 */
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

/**
 * @brief Performs a walking and a driving Dijkstra to find the best path from source to dest that includes driving and walking segments.
 * Total time complexity is O((V+E)logV) where V is the number of vertices and E is the number of edges in the graph.
 * 
 * @tparam T 
 * @param g 
 * @param source vertex where the path starts
 * @param dest vertex where the path ends
 * @param maxWalkTime maximum walking distance allowed
 * @param avoid_nodes nodes that the path can't go through
 * @param avoid_edges edges that the path can't go through
 * @param path used to return the best path as a list of ordered IDs
 * @param parkingNodeId used to return the ID of the node where the user should park
 * @param walkingTime used to return the walking time of the best path
 * @param drivingTime used to return the driving time of the best path
 * @return int is 0 if a path was found, -1 otherwise
 */
template <class T>
int calculateEnvironmentallyFriendlyPath(const Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, const int maxWalkTime, const std::unordered_set<T>& avoid_nodes, const std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges, std::list<T>& path, T& parkingNodeId, int& walkingTime, int& drivingTime) {
    std::unordered_map<Vertex<T>*, double> reacheableWalkingVertices = {};
    std::vector<Vertex<T>*> visitedVertices = {};
    int err = walkingReverseDijsktra(g, dest, source, maxWalkTime, avoid_nodes, avoid_edges, reacheableWalkingVertices, visitedVertices);
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