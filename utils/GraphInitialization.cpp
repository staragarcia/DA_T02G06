#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Graph.h"
#include <limits>

void readParseLocations(Graph<int> &g) {
    std::ifstream file("datasets/locations.csv");
    if (!file.is_open()) {
        std::cout << "Error opening file!\n";
    }
    
    std::string line;
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string location;
        int id;
        std::string code;
        bool parking;
        
        std::getline(ss, location, ',');
        ss >> id;
        ss.ignore(); 
        std::getline(ss, code, ',');
        ss >> parking;
        g.addVertex(location, id, code, parking);
    }
}

void readParseDistances(Graph<int> &g) {
    std::ifstream file("datasets/distances.csv");
    if (!file.is_open()) {
        std::cout << "Error opening file!\n";
    }
    
    std::string line;
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string location1;
        std::string location2;
        std::string driving_str;
        int walking;
        
        std::getline(ss, location1, ',');
        std::getline(ss, location2, ',');
        std::getline(ss, driving_str, ',');
        ss >> walking;

        int driving;
        if (driving_str == "X") {
            driving = std::numeric_limits<int>::max();  
        } else {
            std::stringstream(driving_str) >> driving;
        }
        g.addBidirectionalEdge(location1, location2, driving, walking);
    }
}