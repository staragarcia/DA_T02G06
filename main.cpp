#include <iostream>
#include "utils/Graph.h"
#include "batchmode/BatchMode.cpp" // Include the BatchMode header
#include <string>

using namespace std;

/**
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

    cout << "Enter the source node id: ";
    cin >> sourceId;
    while (!(source = g.findVertexById(sourceId))) {
        cout << "Error: Invalid source. Try again: ";
        cin >> sourceId;
    }

    cout << "Enter the destination node id: ";
    cin >> destinationId;
    while (!(destination = g.findVertexById(destinationId))) {
        cout << "Error: Invalid destination. Try again: ";
        cin >> destinationId;
    }

    cout << "\n";
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
    }
}