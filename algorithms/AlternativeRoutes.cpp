#pragma once

#include "EnvironmentallyFriendly.cpp"

/**
 * @brief Copies path 2, parking node 2, walking time 2, and driving time 2 to path 1, parking node 1,
 * walking time 1 and driving time 1 if path 2 is better (smaller total cost) in O(L) where L is the size of path2.
 * 
 * @tparam T 
 * @param path1 list of node ids representing path 1
 * @param parkingNodeId1 Id of the parking node of path 1
 * @param walkingTime1 walking time of path 1
 * @param drivingTime1 driving time of path 1
 * @param path2 
 * @param parkingNodeId2 
 * @param walkingTime2 
 * @param drivingTime2 
 */
template <class T> 
void copyIfBetter(std::list<T>& path1, T& parkingNodeId1, int& walkingTime1, int& drivingTime1, std::list<T>& path2, T& parkingNodeId2, int& walkingTime2, int& drivingTime2) {
    if (walkingTime1 == std::numeric_limits<int>::max() || drivingTime1 == std::numeric_limits<int>::max() || walkingTime2 + drivingTime2 < walkingTime1 + drivingTime1) {
        path1 = path2;
        parkingNodeId1 = parkingNodeId2;
        walkingTime1 = walkingTime2;
        drivingTime1 = drivingTime2;
    }
}

/**
 * @brief Given a driving-walking path, this function calculates the best alternative path with the same constraints. 
 * The function works by avoiding each of the edges in the path at a time and calculating the best path with the new restriction, using the EnvironmentallyFriendly algorithm which has time complexity of O(log(V)(E+V)).
 * The total complexity is: O(Vlog(V)(E+V)) where V is the number of vertices and E is the number of edges since the maximum number of edges in a path is V-1.
 * 
 * @tparam T 
 * @param g graph where the path is calculated
 * @param source node where the path starts
 * @param dest node where the path ends
 * @param maxWalkTime maximum walking time allowed
 * @param avoid_nodes nodes that the path can't go through
 * @param avoid_edges nodes that the path can't go through
 * @param previousPath path that will be used to calculate the alternative path
 * @param altPath used to return alternative path
 * @param altDrivingTime used to return the driving time of the alternative path
 * @param altWalkingTime used to return the walking time of the alternative path
 * @param altParkingNodeId used to return the parking node id of the alternative path
 */
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

/**
 * @brief Given that no path was found with the constraints, this function tries to remove each constraint one by one and find one path and an alternative to it with the same constraints.
 * The order by which constraints are removed is the following:
 * - Maximum walking time constraint
 * - Edge avoidance constraint
 * - Node avoidance constraint
 * - Maximum walking time and edge avoidance constraints
 * - Maximum walking time and node avoidance constraints
 * - Node and edge avoidance constraints
 * - No constraints, except that the path must include driving and walking segments (must include a parking node which isn't the source or destination)
 * 
 * @tparam T 
 * @param g graph where the path is calculated
 * @param source vertex that the path starts in
 * @param dest vertex where the path ends
 * @param maxWalkTime maximum allowed walking time
 * @param avoid_nodes nodes that the path can't go through
 * @param avoid_edges edges that the path can't go through
 * @param path1 used to return the first alternative path as list of node ids
 * @param parkingNodeId1 used to return the parking node id of the first alternative path
 * @param walkingTime1 used to return the walking time of the first alternative path
 * @param drivingTime1 used to return the driving time of the first alternative path
 * @param path2 used to return the second alternative path as list of node ids
 * @param parkingNodeId2 used to return the parking node id of the second alternative path
 * @param walkingTime2 used to return the walking time of the second alternative path
 * @param drivingTime2 used to return the driving time of the second alternative path
 * @return std::string returns a message indicating the constraints that were removed to find the alternative paths
 */
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