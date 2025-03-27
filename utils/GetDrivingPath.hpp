#pragma once

#include "Graph.h"
#include <list>

/**
 * @brief Read the path that leads to a vertex from the vertices
 * Runs in O(V) time where V is the number of vertices in the graph because the path's length is at most V-1.
 * 
 * @tparam T template parameter related to the type of vertice
 * @param destNode Last vertice on the path
 * @param orderedIds initialized list where the ids of the vertices on the path will be put
 */
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