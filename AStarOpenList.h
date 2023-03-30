//
// Created by Debraj Ray on 2023-03-13.
//

#ifndef INC_658A2_ASTAROPENLIST_H
#define INC_658A2_ASTAROPENLIST_H

#include <iostream>
#include <vector>
#include "Node.h"
#include <unordered_set>
#include <unordered_map>

using namespace std;
typedef unsigned long long ulonglong;

class AStarOpenList {

    /**
     * This is A* open list implementation using min heap. The underlying DS is
     * a vector.
     */
    unique_ptr<vector<Node>> fscoreHeap;
    /**
     * An extra map is maintained for constant time operations on the heap.
     * For example, if we want to know if an element exists in the heap, then this map
     * can do the lookup in constant time.
     */
    unique_ptr<unordered_map<ulonglong, ulonglong>> openList;

    ulonglong size = 0;
    ulonglong maxSize = 0;

    /**
     * Returns the F-score / G-score of the left / right child
     */
    int leftChild_F(ulonglong parentIndex);
    int rightChild_F(ulonglong parentIndex);
    int leftChild_G(ulonglong parentIndex);
    int rightChild_G(ulonglong parentIndex);

    /**
     * Standard Heap Operations
     */
    void sink(ulonglong k);
    void swim(ulonglong k);

    /**
     * Compares two nodes by their f scores. Returns true if the left node's
     * f score is smaller. For tie-breaking we check the g-scores. The lower g-score is
     * preferred.
     */
    inline bool compareFScores(int leftF, int rightF, int leftG, int rightG);

public:

    AStarOpenList() {
        fscoreHeap = make_unique<vector<Node>>();
        /**
         * We need to reserve memory since for this problem since a lot of states are stored in the vector & map.
         * Internally, resize of these STL containers is not constant time. And this grows with size of the containers.
         * Reserving memory ensure insert in the vector / map stays constant time.
         */
        openList = make_unique<unordered_map<ulonglong, ulonglong>>();
    }

    /**
     * Insert a node in the open list
     */
    void insert(Node node);

    void exchange(ulonglong idx1, ulonglong idx2);

    Node removeMinimum();

    int peekMinimum();

    bool isEmpty() const;

    bool isPresent(Node& n);

    bool updateIfBetterPath(Node& n, int gvalue);

    ulonglong getMaxSize();
};


#endif //INC_658A2_ASTAROPENLIST_H
