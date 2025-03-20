#pragma once

#include "Graph.h"
#include <list>

template <class T> 
void getDrivingPath(Vertex<T>* parkingNode, std::list<T>& orderedIds) {
    orderedIds.push_front(parkingNode->getId());
    Edge<T>* aux_edge = parkingNode->getPath();
    while (aux_edge != nullptr) {
        orderedIds.push_front(aux_edge->getOrig()->getId());
        aux_edge = aux_edge->getOrig()->getPath();
    }
}