//
// Created by Debraj Ray on 2023-04-05.
//

#ifndef INC_658PROJECT_ABSTRACTION_H
#define INC_658PROJECT_ABSTRACTION_H

#include "AbstractNode.h"

class Abstraction {

public:

    virtual void setGoalColor(int color) = 0;

    virtual double heuristic(int nodeColor) = 0;

    virtual Node createNode(int color) = 0;

    virtual bool isGoalReached(int color) = 0;

    virtual AbstractNode& unrank(ulonglong rank) = 0;

    virtual int getGoalColor() = 0;

    virtual vector<AbstractNode> getAllAbstractNodes() = 0;

    virtual int getStartColor() = 0;

    virtual double getGCost(const AbstractNode &n1, const AbstractNode &n2) {
        return sqrt(pow(n1.representationCenter.first - n2.representationCenter.first, 2) +
        pow(n1.representationCenter.second - n2.representationCenter.second, 2));
    };

    virtual unordered_map<int, AbstractNode>& accessAbstractGraph() = 0;

    virtual void createAbstractGraphNodes() = 0;
};

#endif //INC_658PROJECT_ABSTRACTION_H
