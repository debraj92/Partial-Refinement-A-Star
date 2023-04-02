//
// Created by Debraj Ray on 2023-03-25.
//

#ifndef INC_658PROJECT_ABSTRACTNODE_H
#define INC_658PROJECT_ABSTRACTNODE_H

#include <iostream>
#include <unordered_set>

using namespace std;

class AbstractNode {

public:

    int color;
    pair<int, int> centroidRealNode;

    AbstractNode(){
    }

    AbstractNode(int color_, pair<int, int> &centroidReal) : color(color_), centroidRealNode(centroidReal) {
    }

    // have a direct edge
    unordered_set<int> reachableNodes{};

    void addChildAbstractNode(int childColor) {
        if (reachableNodes.find(childColor) == reachableNodes.end()) {
            // insert if not inserted already
            reachableNodes.insert(childColor);
        }
    }
};


#endif //INC_658PROJECT_ABSTRACTNODE_H
