//
// Created by Debraj Ray on 2023-03-13.
//

#ifndef INC_658A2_NODE_H
#define INC_658A2_NODE_H

#include <iostream>

using namespace std;
typedef unsigned long long ulonglong;

class Node {
public:

    ulonglong rank;

    double g, h;
    ulonglong heap_idx;

    Node(ulonglong rank_) : rank(rank_) {
        g = -1;
        h = -1;
        heap_idx = -1;
    }

    Node() {
        g = -1;
        h = -1;
        heap_idx = -1;
    }

    // make copy more efficient

    void calculateF(double gScore, double hScore) {
        g = gScore;
        h = hScore;
    }

    double getFScore() const {
        return g + h;
    }

    bool operator==(const Node& other) const
    {
        if (this->rank == other.rank) return true;
        return false;
    }

    bool operator!=(const Node& other) const
    {
        if (this->rank != other.rank) return true;
        return false;
    }

};

#endif //INC_658A2_NODE_H
