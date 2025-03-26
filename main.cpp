#include <iostream>
#include "utils/Graph.h"
#include "batchmode/BatchMode.cpp" // Include the BatchMode header

using namespace std;

/**
 * @brief Displays the main menu for the Route Planning Analysis Tool.
 */
void displayMenu() {
    cout << "\n===== Route Planning Analysis Tool =====\n";
    cout << "1. Find fastest route\n";
    cout << "2. Find second-fastest route\n";
    cout << "3. Plan restricted route\n";
    cout << "4. Plan eco-friendly route\n";
    cout << "5. Run batch mode\n"; // Add batch mode option
    cout << "6. Exit\n";
    cout << "Enter your option: ";
}

void shortestRoute() {
    cout << "Finding shortest route...\n";
    // Add logic for shortest route
}

void secondShortestRoute() {
    cout << "Finding second-shortest route...\n";
    // Add logic for second-shortest route
}

void restrictedRoute() {
    cout << "Planning route with restrictions...\n";
    // Add logic for restricted route
}

void ecoFriendlyRoute() {
    cout << "Planning environmentally friendly route...\n";
    // Add logic for eco-friendly route
}

void runBatchMode() {
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
            case 1: shortestRoute(); break;
            case 2: secondShortestRoute(); break;
            case 3: restrictedRoute(); break;
            case 4: ecoFriendlyRoute(); break;
            case 5: runBatchMode(); break; // Call batch mode
            case 6: cout << "Exiting...\n"; return 0;
            default: cout << "Invalid option! Please try again.\n";
        }
    }
}