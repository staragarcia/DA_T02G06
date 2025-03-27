#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_set>
#include <regex>
#include "../utils/Graph.h"
#include "../utils/RestrictedDijkstra.hpp"
#include "../utils/GetDrivingPath.hpp"
#include "../algorithms/IndependentRoutePlanning.cpp"
#include "../algorithms/RestrictedRoutePlanning.cpp"
#include "../algorithms/EnvironmentallyFriendly.cpp"
#include "../algorithms/AlternativeRoutes.cpp"
#include "../utils/GraphInitialization.cpp"

using namespace std;

/**
 * @brief Outputs the given path and cost to a file.
 * 
 * @param path The list of nodes representing the path.
 * @param cost The total cost of the path.
 * @param outputFile The output file stream.
 */
void outputPathAndCost(list<int>& path, int cost, ofstream& outputFile) {
    if (path.size() < 2 || cost < 0) {
        outputFile << "none\n";
        return;
    }
    bool first = true;
    for (int node : path) {
        if (!first) outputFile << ",";
        outputFile << node;
        first = false;
    }
    outputFile << "(" << cost << ")\n";
}

/**
 * @brief Splits a driving and walking path into two separate lists.
 * 
 * @param path The full path including both driving and walking sections.
 * @param parkingNodeId The node where the switch from driving to walking occurs.
 * @param drivingPath The list to store the driving portion of the path.
 * @param walkingPath The list to store the walking portion of the path.
 */
void parseDrivingWalkingPath(list<int>& path, int parkingNodeId, list<int>& drivingPath, list<int>& walkingPath) {
    bool driving = true;
    for (int node : path) {
        if (node == parkingNodeId) {
            driving = false;
            drivingPath.push_back(node);
            walkingPath.push_back(node);
            continue;
        }
        if (driving) {
            drivingPath.push_back(node);
        } else {
            walkingPath.push_back(node);
        }
    }
}

/**
 * @brief Outputs the driving and walking portions of a path to a file.
 * 
 * @param path The full path including driving and walking sections.
 * @param parkingNodeId The node where the switch from driving to walking occurs.
 * @param outputFile The output file stream.
 * @param drivingTime The time taken for the driving portion.
 * @param walkingTime The time taken for the walking portion.
 * @param mode The mode identifier for the output.
 */
void outputDrivingWalkingPath(list<int>& path, int parkingNodeId, ofstream& outputFile, int& drivingTime, int& walkingTime, string mode) {
    list<int> drivingPath = {};
    list<int> walkingPath = {};
    parseDrivingWalkingPath(path, parkingNodeId, drivingPath, walkingPath);
    if (drivingPath.size() < 2 || walkingPath.size() < 2) {
        outputFile << "DrivingRoute" << mode << ":none\nParkingNode" << mode << ":none\nWalkingRoute" << mode << ":none\nTotalTime" << mode << ":\n";
        return;
    }
    outputFile << "DrivingRoute" << mode << ":";
    outputPathAndCost(drivingPath, drivingTime, outputFile);
    outputFile << "ParkingNode" << mode << ":" << parkingNodeId << "\n";
    outputFile << "WalkingRoute" << mode << ":";
    outputPathAndCost(walkingPath, walkingTime, outputFile);
    outputFile << "TotalTime" << mode << ":" << drivingTime + walkingTime << "\n";
}

/**
 * @brief Processes a batch mode operation for route planning.
 * 
 * Reads input from "batchmode/input.txt", processes the data, and writes the output
 * to "batchmode/output.txt". Supports different routing modes and constraints.
 * 
 * @param graph The graph representing the road network.
 */
void processBatchMode(Graph<int>& graph) {
    // INPUT //
    ifstream inputFile("batchmode/input.txt");
    ofstream outputFile("batchmode/output.txt");

    if (!inputFile) {
        cerr << "Error: Could not open input file.\n";
        return;
    }

    string mode;
    int sourceId = -1, destinationId = -1;
    unordered_set<int> avoidNodes = {};
    unordered_set<pair<int, int>, pairHash> avoidEdges = {};
    int includeNode = -1;
    int maxWalkTime = -1;

    string line;
    while (getline(inputFile, line)) {
        stringstream ss(line);
        string key, value;
        getline(ss, key, ':');
        getline(ss, value);
        if (key == "Mode") {
            if (value != "driving" && value != "driving-walking") {
                outputFile << "Error: Invalid mode. " << value << "\n";
                outputFile.close();
                return;
            }
            mode = value;
        } else if (key == "Source") {
            try {
                sourceId = stoi(value);
            } catch (invalid_argument& e) {
                outputFile << "Error: Invalid source. " << value << "\n";
                outputFile.close();
                return;
            }
        } else if (key == "Destination") {
            try {
                destinationId = stoi(value);
            } catch (invalid_argument& e) {
                outputFile << "Error: Invalid destination. " << value << "\n";
                outputFile.close();
                return;
            }
        } else if (key == "AvoidNodes") {
            stringstream nodesStream(value);
            string node;
            while (getline(nodesStream, node, ',')) {
                try {
                    avoidNodes.insert(stoi(node));
                } catch (invalid_argument& e) {
                    outputFile << "Error: Invalid node to avoid. " << node << "\n";
                    outputFile.close();
                    return;
                }
            }
        } else if (key == "AvoidSegments") {
            regex segmentRegex(R"(\((\d+),(\d+)\))");
            smatch match;
            string::const_iterator searchStart(value.cbegin());
            while (regex_search(searchStart, value.cend(), match, segmentRegex)) {
                try {
                    int from = stoi(match[1]);
                    int to = stoi(match[2]);
                    avoidEdges.insert({from, to});
                    searchStart = match.suffix().first;
                } catch (invalid_argument& e) {
                    outputFile << "Error: Invalid edge to avoid. " << match[1] << "," << match[2] << "\n";
                    outputFile.close();
                    return;
                }
            }
        } else if (key == "IncludeNode") {
            if (value == "") {
                continue;
            }
            try {
                includeNode = stoi(value);
            } catch (invalid_argument& e) {
                outputFile << "Error: Invalid node to include. " << value << "\n";
                outputFile.close();
                return;
            }
        } else if (key == "MaxWalkTime") {
            if (value == "") {
                continue;
            }
            try {
                maxWalkTime = stoi(value);
            } catch (invalid_argument& e) {
                outputFile << "Error: Invalid max walk time. " << value << "\n";
                outputFile.close();
                return;
            }
        }
    }

    inputFile.close();


    // OUTPUT //
    Vertex<int>* source = graph.findVertexById(sourceId);
    if (source == nullptr) {
        outputFile << "Error: Invalid source. " << sourceId << "\n";
        outputFile.close();
        return;
    }
    Vertex<int>* destination = graph.findVertexById(destinationId);
    if (destination == nullptr) {
        outputFile << "Error: Invalid destination. " << destinationId << "\n";
        outputFile.close();
        return;
    }

    outputFile << "Source:" << sourceId << "\n";
    outputFile << "Destination:" << destinationId << "\n";

    if (mode == "driving" && avoidNodes.empty() && avoidEdges.empty() && includeNode == -1) {

        list<int> bestPath = {}, altPath = {};
        int bestTime = -1, altTime = -1;

        IndependentRoutePlanning(graph, source, destination, bestPath, bestTime, altPath, altTime);
        outputFile << "BestDrivingRoute:";
        outputPathAndCost(bestPath, bestTime, outputFile);
        outputFile << "AlternativeDrivingRoute:";
        outputPathAndCost(altPath, altTime, outputFile);

    // RESTRICTED ROUTE PLANNING
    } else if (mode == "driving") {
        list<int> bestPath = {};
        int bestTime = -1;
        int time = RestrictedRoutePlanning(graph, source, destination, avoidNodes, avoidEdges, graph.findVertexById(includeNode), bestPath);
        outputFile << "RestrictedDrivingRoute:";
        outputPathAndCost(bestPath, time, outputFile);

    //ENVIRONMENTALLY-FRIENDLY ROUTE PLANNING
    } else if (mode == "driving-walking" && maxWalkTime != -1) {
        list<int> path = {};
        int parkingNodeId;
        int walkingTime, drivingTime;
        int err = calculateEnvironmentallyFriendlyPath(graph, source, destination, maxWalkTime, avoidNodes, avoidEdges, path, parkingNodeId, walkingTime, drivingTime);
        if (err != 0) {
            outputFile << "DrivingRoute:none\nParkingNode:none\nWalkingRoute:none\nTotalTime:\nMessage:";
            int parkingNodeId1, parkingNodeId2;
            int walkingTime1 = std::numeric_limits<int>::max(), walkingTime2= std::numeric_limits<int>::max(), drivingTime1= std::numeric_limits<int>::max(), drivingTime2= std::numeric_limits<int>::max();
            std::list<int> path1 = {}, path2 = {};
            std::string message = AlternativeRoutes(graph, source, destination, maxWalkTime, avoidNodes, avoidEdges, path1, parkingNodeId1, walkingTime1, drivingTime1, path2, parkingNodeId2, walkingTime2, drivingTime2);
            outputFile << message << "\n";
            outputDrivingWalkingPath(path1, parkingNodeId1, outputFile, drivingTime1, walkingTime1, "1");
            outputDrivingWalkingPath(path2, parkingNodeId2, outputFile, drivingTime2, walkingTime2, "2");
        } else if (err == 0) {
            outputDrivingWalkingPath(path, parkingNodeId, outputFile, drivingTime, walkingTime, "");
        }
    }

    outputFile.close();
    cout << "\nBatch mode processing completed!\nPlease check output.txt \n";
}

