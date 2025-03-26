#pragma once

#include "EnvironmentallyFriendly.cpp"

template <class T> 
void copyIfBetter(std::list<T>& path1, T& parkingNodeId1, int& walkingTime1, int& drivingTime1, std::list<T>& path2, T& parkingNodeId2, int& walkingTime2, int& drivingTime2) {
    if (walkingTime2 + drivingTime2 < walkingTime1 + drivingTime1 || walkingTime1 == std::numeric_limits<int>::max() || drivingTime1 == std::numeric_limits<int>::max()) {
        path1 = path2;
        parkingNodeId1 = parkingNodeId2;
        walkingTime1 = walkingTime2;
        drivingTime1 = drivingTime2;
    }
}

template <class T> 
void getBestAlternative(const Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, const int maxWalkTime, const std::unordered_set<T>& avoid_nodes, std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges, std::list<T>& previousPath, std::list<T>& altPath, int& altDrivingTime, int& altWalkingTime, T& altParkingNodeId) {
    std::vector<std::pair<int,int>> newAvoidEdges= {};
    T node = previousPath.front();
    bool found_alternative = false;
    altDrivingTime = std::numeric_limits<int>::max();
    altWalkingTime = std::numeric_limits<int>::max();
    altPath = {};
    for (T nextNode : previousPath) {
        if (node != nextNode) {
            newAvoidEdges.push_back({node, nextNode});
        }
        node = nextNode;
    }
    for (auto p : newAvoidEdges) {
        if (avoid_edges.find(p) != avoid_edges.end()) {
            continue;
        }
        std::list<T> tempPath = {};
        T tempParkingNodeId;
        int tempWalkingTime = 0;
        int tempDrivingTime = 0;
        avoid_edges.insert(p);
        int err = calculateEnvironmentallyFriendlyPath(g, source, dest, maxWalkTime, avoid_nodes, avoid_edges, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
        if (err == 0) {
            found_alternative = true;
            copyIfBetter(altPath, altParkingNodeId, altWalkingTime, altDrivingTime, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
        }
        avoid_edges.erase(p);
    }
    if (!found_alternative) {
        altPath = {};
        altDrivingTime = -1;
        altWalkingTime = -1;
    }
}

template <class T>
std::string AlternativeRoutes(const Graph<T>& g, Vertex<T>* source, Vertex<T>* dest, const int maxWalkTime, const std::unordered_set<T>& avoid_nodes, std::unordered_set<std::pair<T, T>, pairHash>& avoid_edges, std::list<T>& path1, T& parkingNodeId1, int& walkingTime1, int& drivingTime1,  std::list<T>& path2, T& parkingNodeId2, int& walkingTime2, int& drivingTime2) {
    std::list<T> tempPath = {};
    T tempParkingNodeId;
    int tempWalkingTime = std::numeric_limits<int>::max();
    int tempDrivingTime = std::numeric_limits<int>::max();
    std::unordered_set<std::pair<T, T>, pairHash> emptyAvoidEdges = {};
    std::unordered_set<T> emptyAvoidNodes = {};
    walkingTime1 = drivingTime1 = walkingTime2 = drivingTime2 = std::numeric_limits<int>::max();
    // Try removing maximum walking time constraint
    {
        int err = calculateEnvironmentallyFriendlyPath(g, source, dest, std::numeric_limits<int>::max(), avoid_nodes, avoid_edges, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
        if (err == 0) {
            copyIfBetter(path1, parkingNodeId1, walkingTime1, drivingTime1, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
            getBestAlternative(g, source, dest, std::numeric_limits<int>::max(), avoid_nodes, avoid_edges, path1, path2, drivingTime2, walkingTime2, parkingNodeId2);
            return "Could not find a path with maximum walking time constraint.";
        }
    }
    // Try removing edge avoidance constraint
    {
        int err = calculateEnvironmentallyFriendlyPath(g, source, dest, maxWalkTime, avoid_nodes, emptyAvoidEdges, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
        if (err == 0) {
            copyIfBetter(path1, parkingNodeId1, walkingTime1, drivingTime1, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
            getBestAlternative(g, source, dest, maxWalkTime, avoid_nodes, emptyAvoidEdges, path1, path2, drivingTime2, walkingTime2, parkingNodeId2);
            return "Could not find a path with edge avoidance constraint.";
        }
    }
    // Try removing node avoidance constraint
    {
        int err = calculateEnvironmentallyFriendlyPath(g, source, dest, maxWalkTime, emptyAvoidNodes, avoid_edges, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
        if (err == 0) {
            copyIfBetter(path1, parkingNodeId1, walkingTime1, drivingTime1, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
            getBestAlternative(g, source, dest, maxWalkTime,emptyAvoidNodes, avoid_edges, path1, path2, drivingTime2, walkingTime2, parkingNodeId2);
            return "Could not find a path with node avoidance constraint.";
        }
    }
    // Try removing maximum walking time and edge avoidance constraints
    {
        int err = calculateEnvironmentallyFriendlyPath(g, source, dest, std::numeric_limits<int>::max(), avoid_nodes, emptyAvoidEdges, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
        if (err == 0) {
            copyIfBetter(path1, parkingNodeId1, walkingTime1, drivingTime1, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
            getBestAlternative(g, source, dest, std::numeric_limits<int>::max(), avoid_nodes, emptyAvoidEdges, path1, path2, drivingTime2, walkingTime2, parkingNodeId2);
            return "Could not find a path with maximum walking time or edge avoidance constraint.";
        }
    }
    // Try removing maximum walking time and node avoidance constraints
    {
        int err = calculateEnvironmentallyFriendlyPath(g, source, dest, std::numeric_limits<int>::max(), emptyAvoidNodes, avoid_edges, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
        if (err == 0) {
            copyIfBetter(path1, parkingNodeId1, walkingTime1, drivingTime1, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
            getBestAlternative(g, source, dest, std::numeric_limits<int>::max(), emptyAvoidNodes, avoid_edges, path1, path2, drivingTime2, walkingTime2, parkingNodeId2);
            return "Could not find a path with maximum walking time or node avoidance constraint.";
        }
    }
    // Try removing node and edge avoidance constraints
    {
        int err = calculateEnvironmentallyFriendlyPath(g, source, dest, maxWalkTime, emptyAvoidNodes, emptyAvoidEdges, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
        if (err == 0) {
            copyIfBetter(path1, parkingNodeId1, walkingTime1, drivingTime1, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
            getBestAlternative(g, source, dest, maxWalkTime,emptyAvoidNodes, emptyAvoidEdges, path1, path2, drivingTime2, walkingTime2, parkingNodeId2);
            return "Could not find a path with node or edge avoidance constraint.";
        }
    }
    // Try removing all constraints
    {
        int err = calculateEnvironmentallyFriendlyPath(g, source, dest, std::numeric_limits<int>::max(), emptyAvoidNodes, emptyAvoidEdges, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
        if (err == 0) {
            copyIfBetter(path1, parkingNodeId1, walkingTime1, drivingTime1, tempPath, tempParkingNodeId, tempWalkingTime, tempDrivingTime);
            getBestAlternative(g, source, dest, std::numeric_limits<int>::max(), emptyAvoidNodes, emptyAvoidEdges, path1, path2, drivingTime2, walkingTime2, parkingNodeId2);
            return "Could not find a path with maximum walking time, or node, or edge avoidance constraint.";
        }
    }
    return "Could not find any path walking and driving from source to destination.";
}