#include <iostream>
#include "utils/Graph.h"
#include "batchmode/BatchMode.cpp" // Include the BatchMode header
#include <string>
#include <regex>

using namespace std;

/**#include <regex>
 * @brief Displays the main menu for the Route Planning Analysis Tool.
 */
void displayMenu() {
    cout << "\n===== Route Planning Analysis Tool =====\n";
    cout << "1. Independent Route Planning\n";
    cout << "2. Restricted Route Planning\n";
    cout << "3. Environmentally-Friendly Route Planning (driving and walking)\n";
    cout << "4. Run batch mode\n"; // Add batch mode option
    cout << "5. Exit\n";
    cout << "Enter your option: ";
}

void validateSourceAndDest (Graph<int> &g, Vertex<int>* &source, Vertex<int>* &destination) {
    int sourceId;
    int destinationId;

    cout << "Source:";
    cin >> sourceId;
    while (!(source = g.findVertexById(sourceId))) {
        cout << "Error: Invalid source. Try again: ";
        cin >> sourceId;
    }

    cout << "Destination:";
    cin >> destinationId;
    while (!(destination = g.findVertexById(destinationId))) {
        cout << "Error: Invalid destination. Try again: ";
        cin >> destinationId;
    }
}

void independentRoute(Graph<int> &g) {
    cout << "Finding shortest route...\n";

    Vertex<int>* source = nullptr;
    Vertex<int>* destination = nullptr;

    validateSourceAndDest(g, source, destination);

    list<int> bestPath = {}, altPath = {};
    int bestTime = -1, altTime = -1;

    // Find the Best Route
    IndependentRoutePlanning(g, source, destination, bestPath, bestTime, altPath, altTime);

    // Print Best Route
    cout << "BestDrivingRoute:";
    outputPathAndCost(bestPath, bestTime, cout);
    // Print Alternative Route
    cout << "AlternativeDrivingRoute:";
    outputPathAndCost(altPath, altTime, cout);
}

void restrictedRoute(Graph<int> &g) {
    cout << "Finding second-shortest route...\n";

    Vertex<int>* source = nullptr;
    Vertex<int>* destination = nullptr;

    validateSourceAndDest(g, source, destination);
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

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
    int time = RestrictedRoutePlanning(g, source, destination, avoidNodes, avoidEdges, g.findVertexById(includeNode), bestPath);
    cout << "RestrictedDrivingRoute:";
    outputPathAndCost(bestPath, time, cout);
    
}

void EFriendlyRoute(Graph<int> &g) {
    cout << "Planning route with restrictions...\n";
    // Add logic for restricted route
}

void runBatchMode(Graph<int> &g) {
    cout << "\n[ Running batch mode... ]\n";
    Graph<int> graph;

    // Initialize the graph
    readParseLocations(graph);
    readParseDistances(graph);

    // Process batch mode
    processBatchMode(graph);
}


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