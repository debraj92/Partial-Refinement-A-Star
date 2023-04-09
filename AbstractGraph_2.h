//
// Created by Debraj Ray on 2023-04-04.
//

#ifndef INC_658PROJECT_ABSTRACTGRAPH_2_H
#define INC_658PROJECT_ABSTRACTGRAPH_2_H

#include <vector>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include "AbstractNode.h"
#include "AbstractGraph.h"
#include "Abstraction.h"

using namespace std;

/**
 * This is a higher level abstraction on top of AbstractGraph.
 * It uses clique based abstraction where each clique can contain MAX 4 nodes
 */
class AbstractGraph_2 : public Abstraction {

    unordered_map<int, AbstractNode> colorAbstractNodeMap;

    AbstractGraph &abGraph;
    RealWorld &rworld;

    int goalColor;

    void dfsToConnectAbstractNodes(const AbstractNode &abNode, int color, unordered_set<int> visited);

    void createUndirectedEdge(int color1, int color2);

    void createAbstractGraphNodes();

    void createUndirectedEdges();

    void printNode(int color);

public:

    AbstractGraph_2(RealWorld &realWorld, AbstractGraph &abG1) : rworld(realWorld), abGraph(abG1) {

    }

    void setGoalColor(int color);

    Node createNode(int color);

    bool isGoalReached(int color);

    AbstractNode& unrank(ulonglong rank);

    int getGoalColor();

    vector<AbstractNode> getAllAbstractNodes();

    void createAbstractGraph();

    int getStartColor();

    unordered_map<int, AbstractNode>& accessAbstractGraph();
};


#endif //INC_658PROJECT_ABSTRACTGRAPH_2_H
