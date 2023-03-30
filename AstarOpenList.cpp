//
// Created by Debraj Ray on 2023-03-13.
//

#include "AStarOpenList.h"

int AStarOpenList::leftChild_F(ulonglong parentIndex) {
    return (*fscoreHeap)[2*parentIndex + 1].getFScore();
}

int AStarOpenList::rightChild_F(ulonglong parentIndex) {
    return (*fscoreHeap)[2*parentIndex + 2].getFScore();
}

int AStarOpenList::leftChild_G(ulonglong parentIndex) {
    return (*fscoreHeap)[2*parentIndex + 1].g;
}

int AStarOpenList::rightChild_G(ulonglong parentIndex) {
    return (*fscoreHeap)[2*parentIndex + 2].g;
}

/**
 * Heap operation that can shift element at index k towards the bottom of the tree. Time Complexity : O(logN)
 */
void AStarOpenList::sink(ulonglong k) {
    ulonglong leftChildIdx, rightChildIdx, minChildIdx;
    while(2*k + 1 <size) {
        leftChildIdx = 2*k + 1;
        rightChildIdx = 2*k + 2;
        if (rightChildIdx < size) {
            minChildIdx = rightChildIdx;
            // tie-breaking with g
            if(compareFScores(
                    leftChild_F(k),
                    rightChild_F(k),
                    leftChild_G(k),
                    rightChild_G(k))) {

                minChildIdx = leftChildIdx;
            }
            // tie-breaking with g
            if(compareFScores((*fscoreHeap)[minChildIdx].getFScore(),
                              (*fscoreHeap)[k].getFScore(),
                              (*fscoreHeap)[minChildIdx].g,
                              (*fscoreHeap)[k].g))
            {
                exchange(minChildIdx, k);
                k = minChildIdx;
            } else {
                // already a heap
                break;
            }
        } else {
            // tie-breaking with g
            if(compareFScores(leftChild_F(k),
                              (*fscoreHeap)[k].getFScore(),
                              leftChild_G(k),
                              (*fscoreHeap)[k].g)) {
                exchange(leftChildIdx, k);
                k = leftChildIdx;
            } else {
                // already a heap
                break;
            }
        }
    }
}

/**
 * Swaps element at index idx1 with idx2 in the heap. Time Complexity : O(1)
 */
void AStarOpenList::exchange(ulonglong idx1, ulonglong idx2) {
    if (idx1 == idx2) {
        return;
    }
    std::swap((*fscoreHeap)[idx1], (*fscoreHeap)[idx2]);

    (*fscoreHeap)[idx1].heap_idx = idx1;
    (*fscoreHeap)[idx2].heap_idx = idx2;

    // insert updated heap values to open list
    (*openList)[(*fscoreHeap)[idx1].rank] = idx1;
    (*openList)[(*fscoreHeap)[idx2].rank] = idx2;
}

/**
 * Heap operation that can shift element at index k towards the top of the tree. Time Complexity : O(logN)
 */
void AStarOpenList::swim(ulonglong k) {
    ulonglong parent_index;
    while (k>=1) {
        parent_index = (k - 1) / 2;
        // tie-breaking with g
        if(compareFScores((*fscoreHeap)[k].getFScore(),
                             (*fscoreHeap)[parent_index].getFScore(),
                             (*fscoreHeap)[k].g,
                             (*fscoreHeap)[parent_index].g)) {

            exchange(k, parent_index);
            k = parent_index;
        } else {
            break;
        }
    }
}

/**
 * Insert a new node in the heap. O(logN) where N is the number of elements in the heap
 */
void AStarOpenList::insert(Node node) {
    node.heap_idx = size;
    if(size >= (*fscoreHeap).size()) {
        fscoreHeap->push_back(node);
    } else {
        (*fscoreHeap)[size] = node;
    }
    openList->insert({node.rank, size});
    size++;
    maxSize = size > maxSize ? size : maxSize;
    swim(size-1);
}

/**
 * Removes and returns the smallest element from the heap. Also re-arranges the elements to ensure
 * heap property is maintained. O(logN) where N is the number of elements in the heap
 */
Node AStarOpenList::removeMinimum() {
    exchange(0, size-1);
    Node fscore_deleted = (*fscoreHeap)[size-1];
    size--;
    sink(0);
    (*openList)[fscore_deleted.rank] = -1;
    return fscore_deleted;
}

int AStarOpenList::peekMinimum() {
    return (*fscoreHeap)[0].getFScore();
}

/**
 * Checks if heap is empty in constant time.
 */
bool AStarOpenList::isEmpty() const {
    return size == 0;
}

/**
 * Checks if a node exists in the heap. Constant time operation.
 */
bool AStarOpenList::isPresent(Node& n) {
    const auto& it = openList->find(n.rank);
    return it != openList->end() && it->second != -1;
}

/**
 * Update g cost of an existing node if a better g cost is found through another path.
 * Time Complexity: O(logN)
 */
bool AStarOpenList::updateIfBetterPath(Node& n, int gvalue) {
    auto idx = openList->find(n.rank)->second;
    if (gvalue < (*fscoreHeap)[idx].g) {
        n.calculateF(gvalue, (*fscoreHeap)[idx].h);
        n.heap_idx = (*fscoreHeap)[idx].heap_idx;
        (*fscoreHeap)[idx] = n;
        (*openList)[(*fscoreHeap)[idx].rank] = idx;
        swim(n.heap_idx);
        return true;
    }
    return false;
}

ulonglong AStarOpenList::getMaxSize() {
    return maxSize;
}

inline bool AStarOpenList::compareFScores(int leftF, int rightF, int leftG, int rightG) {
    return leftF == rightF ? (leftG < rightG) : (leftF < rightF);
}
