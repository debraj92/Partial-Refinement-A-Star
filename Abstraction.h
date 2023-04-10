//
// Created by Debraj Ray on 2023-04-05.
//

#ifndef INC_658PROJECT_ABSTRACTION_H
#define INC_658PROJECT_ABSTRACTION_H

#include "AbstractNode.h"

class Abstraction {

public:

    int goalX, goalY;

    virtual void setGoalColor(int color) = 0;

    virtual Node createNode(int color) = 0;

    virtual bool isGoalReached(int color) = 0;

    virtual AbstractNode& unrank(ulonglong rank) = 0;

    virtual int getGoalColor() = 0;

    virtual vector<AbstractNode> getAllAbstractNodes() = 0;

    virtual int getStartColor() = 0;

    virtual inline double getGCost(const AbstractNode &n1, const AbstractNode &n2) {
        return sqrt(pow(n1.representationCenter.first - n2.representationCenter.first, 2) +
        pow(n1.representationCenter.second - n2.representationCenter.second, 2));
    };

    virtual unordered_map<int, AbstractNode>& accessAbstractGraph() = 0;

    virtual void createAbstractGraphNodes() = 0;

    void setGoal(int goalx, int goaly) {
        this->goalX = goalx;
        this->goalY = goaly;
    }

    virtual double heuristic(int nodeColor) {
        const auto &node = unrank(nodeColor);
        const auto& representationCurrent = node.representationCenter;
        double delta_x = abs(representationCurrent.first - goalX);
        double delta_y = abs(representationCurrent.second - goalY);

        return (abs(delta_x - delta_y) + sqrt(2 * delta_x * delta_y)) * (1 - ((double) node.totalNodesInRepresentation / 32768));
    };

    virtual double findShortestDistanceBetweenNodes(const AbstractNode &node1, const AbstractNode &node2) {
        const auto &n1 = node1.representationCenter;
        const auto &n2 = node2.representationCenter;
        return sqrt(pow(n1.first - n2.first, 2) + pow(n1.second - n2.second,2));
    }

};

#endif //INC_658PROJECT_ABSTRACTION_H
