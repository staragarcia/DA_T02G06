#include <iostream>
using namespace std;

void displayMenu() {
    cout << "\n===== Route Planning Analysis Tool =====\n";
    cout << "1. Find fastest route\n";
    cout << "2. Find second-fastest route\n";
    cout << "3. Plan restricted route\n";
    cout << "4. Plan eco-friendly route\n";
    cout << "5. Exit\n";
    cout << "Enter your option: ";
}

void shortestRoute() {
    cout << "Finding shortest route...\n";
}

void secondShortestRoute() {
    cout << "Finding second-shortest route...\n";
}

void restrictedRoute() {
    cout << "Planning route with restrictions...\n";
}

void ecoFriendlyRoute() {
    cout << "Planning environmentally friendly route...\n";
}

int main() {
    int option;
    while (true) {
        displayMenu();
        cin >> option;

        if (cin.fail()) {  // Handle invalid input (e.g., letters instead of numbers)
            cin.clear();
            cin.ignore(10000, '\n');
            cout << "Invalid input! Please enter a number between 1 and 5.\n";
            continue;
        }

        switch (option) {
            case 1: shortestRoute(); break;
            case 2: secondShortestRoute(); break;
            case 3: restrictedRoute(); break;
            case 4: ecoFriendlyRoute(); break;
            case 5: cout << "Exiting...\n"; return 0;
            default: cout << "Invalid option! Please try again.\n";
        }
    }
}