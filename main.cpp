#include <iostream>
#include "utils/Graph.h"
#include "batchmode/BatchMode.cpp" // Include the BatchMode header
#include <string>
#include <regex>

using namespace std;

/**
 * @brief Displays the main menu for the Route Planning Analysis Tool. This function prints the available options for the user to interact with the tool.
 *
 */
void displayMenu() {
    cout << "\n=====| Route Planning Analysis Tool |=====\n";
    cout << "1. Independent Route Planning\n";
    cout << "2. Restricted Route Planning\n";
    cout << "3. Environmentally-Friendly Route Planning (driving and walking)\n";
    cout << "4. Run batch mode\n"; // Add batch mode option
    cout << "5. Exit\n";
    cout << "Enter your option: ";
}

/**
 * @brief Verifies if a string is fully numeric, for parsing purposes.
 * 
 * @param str 
 * @return true 
 * @return false 
 */
bool is_numeric(const std::string& str) {
    return !str.empty() && std::all_of(str.begin(), str.end(), ::isdigit);
}

/**
 * @brief Reads the source and destination vertices from user input. This function prompts the user to input the IDs of the source and destination vertices. It validates the input to ensure the vertices exist in the graph.
 * 
 * @param g Reference to the graph object.
 * @param source Reference to a pointer where the source vertex will be stored.
 * @param destination Reference to a pointer where the destination vertex will be stored.
 */
void readSourceAndDest (Graph<int> &g, Vertex<int>* &source, Vertex<int>* &destination) {
    std::string sourceIdString, destinationIdString;
    int sourceId, destinationId;

    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    while (true) {
        std::cout << "Source:";
        std::getline(std::cin, sourceIdString);

        if (!is_numeric(sourceIdString)) {
            std::cout << "Invalid input! Please enter a number.\n";
            continue;
        }

        try {
            sourceId = std::stoi(sourceIdString);  // Attempt conversion

            if (!(source = g.findVertexById(sourceId))) {
                std::cout << "Error: Invalid source. Try again.\n";
                continue;
            }

            break;  // Valid input, exit loop

        } catch (const std::out_of_range&) {   // Input is above the int limit
            std::cout << "Number out of range! Please enter a smaller number.\n";
            continue;
        }
    }

    while (true) {
        std::cout << "Destination:";
        std::getline(std::cin, destinationIdString);

        if (!is_numeric(destinationIdString)) {
            std::cout << "Invalid input! Please enter a number.\n";
            continue;
        }

        try {
            destinationId = std::stoi(destinationIdString);  

            if (!(destination = g.findVertexById(destinationId))) {
                std::cout << "Error: Invalid destination. Try again.\n";
                continue;
            }

            break;  

        } catch (const std::out_of_range&) {
            std::cout << "Number out of range! Please enter a smaller number.\n";
            continue;
        }
    }
}

/**
 * @brief Executes the independent route planning functionality.
 * 
 * @param g Reference to the graph object.
 */
void independentRoute(Graph<int> &g) {
    cout << "Finding best and alternative routes...\n";

    Vertex<int>* source = nullptr;
    Vertex<int>* destination = nullptr;

    readSourceAndDest(g, source, destination);

    list<int> bestPath = {}, altPath = {};
    int bestTime = -1, altTime = -1;

    // Find the Best Route
    IndependentRoutePlanning(g, source, destination, bestPath, bestTime, altPath, altTime);

    cout << "\n========| OUTPUT |========\n";
    // Print Source and Destination
    outputSourceDest(source->getId(), destination->getId(), cout);
    // Print Best Route
    cout << "BestDrivingRoute:";
    outputPathAndCost(bestPath, bestTime, cout);
    // Print Alternative Route
    cout << "AlternativeDrivingRoute:";
    outputPathAndCost(altPath, altTime, cout);
}

/**
 * @brief Executes the restricted route planning functionality.
 * 
 * @param g Reference to the graph object.
 */
void restrictedRoute(Graph<int> &g) {
    cout << "Finding restricted route...\n";

    Vertex<int>* source = nullptr;
    Vertex<int>* destination = nullptr;

    readSourceAndDest(g, source, destination);

    unordered_set<int> avoidNodes = {};
    std::string input;

    cout << "AvoidNodes:";
    std::getline(std::cin, input); 

    if (!input.empty()) {
        stringstream ss(input);
        string current;

        while (std::getline(ss, current, ',')) {
            avoidNodes.insert(stoi(current));
        }
    }
        
    unordered_set<pair<int, int>, pairHash> avoidEdges = {};
    
    cout << "AvoidSegments:";
    std::getline(std::cin, input); 

    if (!input.empty()) {
        regex segmentRegex(R"(\((\d+),(\d+)\))");
        smatch match;
        string::const_iterator searchStart(input.cbegin());
        while (regex_search(searchStart, input.cend(), match, segmentRegex)) {
            try {
                int from = stoi(match[1]);
                int to = stoi(match[2]);
                avoidEdges.insert({from, to});
                searchStart = match.suffix().first;
            } catch (invalid_argument& e) {
                cout << "Error: Invalid edge to avoid. " << match[1] << "," << match[2] << "\n";
                return;
            }
        }
    }

    int includeNode;

    cout << "IncludeNode:";
    std::getline(std::cin, input); 
    
    if (!input.empty()) {
        includeNode = stoi(input);
    }
    
    list<int> bestPath = {};
    int bestTime = -1;
    cout << "\n========| OUTPUT |========\n";
    outputSourceDest(source->getId(), destination->getId(), cout);
    int time = RestrictedRoutePlanning(g, source, destination, avoidNodes, avoidEdges, g.findVertexById(includeNode), bestPath);
    cout << "RestrictedDrivingRoute:";
    outputPathAndCost(bestPath, time, cout);
    
}

/**
 * @brief Executes the environmentally-friendly route planning functionality.
 * 
 * @param g Reference to the graph object.
 */
void EFriendlyRoute(Graph<int> &g) {
    cout << "Finding environmentally-friendly route...\n";

    Vertex<int>* source = nullptr;
    Vertex<int>* destination = nullptr;

    readSourceAndDest(g, source, destination);

    int maxWalkTime;
    std::string input;

    cout << "MaxWalkTime:";
    std::getline(std::cin, input); 
    
    if (input.empty()) maxWalkTime = std::numeric_limits<int>::max();
    else maxWalkTime = stoi(input);

    unordered_set<int> avoidNodes = {};

    cout << "AvoidNodes:";
    std::getline(std::cin, input); 

    if (!input.empty()) {
        stringstream ss(input);
        string current;

        while (std::getline(ss, current, ',')) {
            avoidNodes.insert(stoi(current));
        }
    }
        
    unordered_set<pair<int, int>, pairHash> avoidEdges = {};
    
    cout << "AvoidSegments:";
    std::getline(std::cin, input); 

    if (!input.empty()) {
        regex segmentRegex(R"(\((\d+),(\d+)\))");
        smatch match;
        string::const_iterator searchStart(input.cbegin());
        while (regex_search(searchStart, input.cend(), match, segmentRegex)) {
            try {
                int from = stoi(match[1]);
                int to = stoi(match[2]);
                avoidEdges.insert({from, to});
                searchStart = match.suffix().first;
            } catch (invalid_argument& e) {
                cout << "Error: Invalid edge to avoid. " << match[1] << "," << match[2] << "\n";
                return;
            }
        }
    }

    list<int> path = {};
    int parkingNodeId;
    int walkingTime, drivingTime;
    cout << "\n========| OUTPUT |========\n";
    outputSourceDest(source->getId(), destination->getId(), cout);
    int err = calculateEnvironmentallyFriendlyPath(g, source, destination, maxWalkTime, avoidNodes, avoidEdges, path, parkingNodeId, walkingTime, drivingTime);
    if (err != 0) {
        cout << "DrivingRoute:\nParkingNode:\nWalkingRoute:\nTotalTime:\nMessage:";
        int parkingNodeId1, parkingNodeId2;
        int walkingTime1 = std::numeric_limits<int>::max(), walkingTime2= std::numeric_limits<int>::max(), drivingTime1= std::numeric_limits<int>::max(), drivingTime2= std::numeric_limits<int>::max();
        std::list<int> path1 = {}, path2 = {};
        std::string message = AlternativeRoutes(g, source, destination, maxWalkTime, avoidNodes, avoidEdges, path1, parkingNodeId1, walkingTime1, drivingTime1, path2, parkingNodeId2, walkingTime2, drivingTime2);
        cout << message << "\n";
        outputDrivingWalkingPath(path1, parkingNodeId1, cout, drivingTime1, walkingTime1, "1");
        outputDrivingWalkingPath(path2, parkingNodeId2, cout, drivingTime2, walkingTime2, "2");
    } else if (err == 0) {
        outputDrivingWalkingPath(path, parkingNodeId, cout, drivingTime, walkingTime, "");
    }
}

/**
 * @brief Executes the batch mode functionality.
 * 
 * @param g Reference to the graph object.
 */
void runBatchMode(Graph<int> &g) {
    cout << "\n[ Running batch mode... ]\n";
    Graph<int> graph;

    // Initialize the graph
    readParseLocations(graph);
    readParseDistances(graph);

    // Process batch mode
    processBatchMode(graph);
}

/**
 * @brief Main function and entry point of the program.
 * 
 * This function initializes the graph, displays the main menu, and handles user input
 * to execute the selected route planning functionality. The program runs in a loop until
 * the user chooses to exit.
 * 
 * @return int Returns 0 upon successful execution.
 */
int main() {
    int option;
    Graph<int> graph;

    // Initialize the graph
    readParseLocations(graph);
    readParseDistances(graph);

    while (true) {
        displayMenu();
        cin >> option;

        if (cin.fail()) {  // Handle invalid input (e.g., letters instead of numbers)
            cin.clear();
            cin.ignore(10000, '\n');
            cout << "Invalid input! Please enter a number between 1 and 6.\n";
            continue;
        }

        switch (option) {
            case 1: independentRoute(graph); break;
            case 2: restrictedRoute(graph); break;
            case 3: EFriendlyRoute(graph); break;
            case 4: runBatchMode(graph); break; // Call batch mode
            case 5: cout << "Exiting...\n"; return 0;
            default: cout << "Invalid option! Please try again.\n";
        }
        cout << "\n";
    }
}