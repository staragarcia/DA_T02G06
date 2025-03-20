#pragma once

#include "Graph.h"
#include <list>

template <class T> 
void getDrivingPath(Vertex<T>* destNode, std::list<T>& orderedIds) {
    if (destNode == nullptr) {
        orderedIds = {};
        return;
    }
    orderedIds.push_front(destNode->getId());
    Edge<T>* aux_edge = destNode->getPath();
    while (aux_edge != nullptr) {
        orderedIds.push_front(aux_edge->getOrig()->getId());
        aux_edge = aux_edge->getOrig()->getPath();
    }
}