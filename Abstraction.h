//
// Created by Debraj Ray on 2023-04-05.
//

#ifndef INC_658PROJECT_ABSTRACTION_H
#define INC_658PROJECT_ABSTRACTION_H

#include "AbstractNode.h"

class Abstraction {

public:

    int goalX, goalY, solutionLength;

    virtual void setGoalColor(ulonglong color) = 0;

    virtual Node createNode(ulonglong color) = 0;

    virtual bool isGoalReached(ulonglong color) = 0;

    virtual AbstractNode& unrank(ulonglong rank) = 0;

    virtual ulonglong getGoalColor() = 0;

    virtual vector<AbstractNode> getAllAbstractNodes() = 0;

    virtual ulonglong getStartColor() = 0;

    virtual inline double getGCost(const AbstractNode &n1, const AbstractNode &n2) {

        return sqrt(pow(n1.representationCenter.first - n2.representationCenter.first, 2) +
                    pow(n1.representationCenter.second - n2.representationCenter.second, 2));
    };

    virtual unordered_map<ulonglong, AbstractNode>& accessAbstractGraph() = 0;

    virtual void createAbstractGraphNodes() = 0;

    void setGoal(int goalx, int goaly) {
        this->goalX = goalx;
        this->goalY = goaly;
    }

    virtual double heuristic(ulonglong nodeColor) {
        const auto &node = unrank(nodeColor);
        /*
        const auto& representationCurrent = node.representationCenter;
        double delta_x = abs(representationCurrent.first - goalX);
        double delta_y = abs(representationCurrent.second - goalY);

        return abs(delta_x - delta_y) + sqrt(2 * delta_x * delta_y);
        */

        return sqrt(pow(node.representationCenter.first - goalX, 2) +
                    pow(node.representationCenter.second - goalY, 2));

    };

    virtual inline double findShortestDistanceBetweenNodes(const AbstractNode &node1, const AbstractNode &node2) {
        return sqrt(pow(node1.representationCenter.first - node2.representationCenter.first, 2) +
        pow(node1.representationCenter.second - node2.representationCenter.second,2));
    }

    void setSolutionLength(int solnLength) {
        this->solutionLength = solnLength;
    }

    int getSolutionLength() {
        return this->solutionLength;
    }

};

#endif //INC_658PROJECT_ABSTRACTION_H
