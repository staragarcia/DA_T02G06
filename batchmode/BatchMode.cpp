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
#include "../utils/GraphInitialization.cpp"

using namespace std;


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
    unordered_set<int> avoidNodes;
    unordered_set<pair<int, int>, pairHash> avoidEdges;
    int includeNode = -1;

    string line;
    while (getline(inputFile, line)) {
        stringstream ss(line);
        string key, value;
        getline(ss, key, ':');
        getline(ss, value);

        if (key == "Mode") {
            mode = value;
        } else if (key == "Source") {
            sourceId = stoi(value);
        } else if (key == "Destination") {
            destinationId = stoi(value);
        }
    }

    inputFile.close();


    // OUTPUT //
    Vertex<int>* source = graph.findVertexById(sourceId);
    Vertex<int>* destination = graph.findVertexById(destinationId);

    if (!source || !destination) {
        outputFile << "Error: Invalid source or destination.\n";
        outputFile.close();
        return;
    }

    outputFile << "Source:" << sourceId << "\n";
    outputFile << "Destination:" << destinationId << "\n";


    // INDEPENDENT ROUTE PLANNING //
    if (mode == "driving" && avoidNodes.empty() && avoidEdges.empty() && includeNode == -1) {

        list<int> bestPath, altPath;
        unordered_set<int> usedNodes;
        int bestTime = -1, altTime = -1;

        // Find the Best Route
        bestPath = IndependentRoutePlanning(graph, source, destination);
        bestTime = RestrictedDijkstra(graph, source, destination, {}, {{}});

        for (int node : bestPath) {
            if (node != sourceId && node != destinationId) {
                usedNodes.insert(node);
            }
        }

        // Find the Alternative Route
        altTime = RestrictedDijkstra(graph, source, destination, usedNodes, {{}});
        getDrivingPath(destination, altPath);

        
        // Print Best Route
        outputFile << "BestDrivingRoute:";
        bool first = true;
        for (int node : bestPath) {
            if (!first) outputFile << ",";
            outputFile << node;
            first = false;
        }
        outputFile << "(" << bestTime << ")\n";

        // Print Alternative Route
        outputFile << "AlternativeDrivingRoute:";
        first = true;
        for (int node : altPath) {
            if (!first) outputFile << ",";
            outputFile << node;
            first = false;
        }
        outputFile << "(" << altTime << ")\n";
    }
    

    outputFile.close();
    cout << "\nBatch mode processing completed!\nPlease check output.txt \n";
}

