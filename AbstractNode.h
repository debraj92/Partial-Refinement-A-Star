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

    /**
     * Color of this node from child node's perspective.
     * It decides the color and paints the child nodes.
     */
    int color;
    /**
     * Color of this node from parent node's perspective.
     * Parent node sets the color
     */
    int abstractionColor = -1;
    pair<int, int> centroidRealNode;

    /**
     * This is not an actual location. Its just the average of the nodes used for representing an
     * abstract region. Should be used to calculate G and H costs in abstract search.
     */
    pair<double, double> representationCenter;
    int totalNodesInRepresentation;

    AbstractNode(){
    }

    AbstractNode(int color_, pair<int, int> &centroidReal, pair<double, double> &centerRepresentation, int totalNodes) :
    color(color_), centroidRealNode(centroidReal), representationCenter(centerRepresentation), totalNodesInRepresentation(totalNodes) {
    }

    // have a direct edge
    unordered_set<int> reachableNodes{};

    void addChildAbstractNode(int childColor) {
        if (!reachableNodes.contains(childColor)) {
            // insert if not inserted already
            reachableNodes.insert(childColor);
        }
    }

    double getXTotalInRepresentation() {
        return totalNodesInRepresentation * representationCenter.first;
    }

    double getYTotalInRepresentation() {
        return totalNodesInRepresentation * representationCenter.second;
    }
};


#endif //INC_658PROJECT_ABSTRACTNODE_H
