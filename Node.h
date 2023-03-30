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

    int g, h;
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

    void calculateF(int gScore, int hScore) {
        g = gScore;
        h = hScore;
    }

    int getFScore() const {
        return g + h;
    }

    struct node_Hash
    {
        size_t operator()(const Node& n) const
        {
            return std::hash<long>()(n.rank);
        }
    };

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
