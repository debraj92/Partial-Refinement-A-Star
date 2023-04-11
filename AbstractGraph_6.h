//
// Created by Debraj Ray on 2023-04-10.
//

#ifndef INC_658PROJECT_ABSTRACTGRAPH_6_H
#define INC_658PROJECT_ABSTRACTGRAPH_6_H

#include <vector>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include "AbstractNode.h"
#include "AbstractGraph_2.h"
#include "AbstractGraph_3.h"
#include "AbstractGraph_4.h"
#include "AbstractGraph_5.h"
#include "AbstractGraph.h"
#include "Abstraction.h"

using namespace std;

class AbstractGraph_6 : public Abstraction {

    const int MAX_EDGE_LENGTH = 32;
    const int MAX_NODES = (MAX_EDGE_LENGTH * 10)^2;

    unordered_map<ulonglong, AbstractNode> colorAbstractNodeMap;

    AbstractGraph_5 &abGraph5;
    AbstractGraph_4 &abGraph4;
    AbstractGraph_3 &abGraph3;
    AbstractGraph_2 &abGraph2;
    AbstractGraph &abGraph;
    RealWorld &rworld;

    ulonglong goalColor;

    void dfsToConnectAbstractNodes(const AbstractNode &abNode, ulonglong abG6Color, unordered_set<ulonglong> &visited);

    void createUndirectedEdge(ulonglong color1, ulonglong color2);

    void createAbstractGraphNodes();

    void createUndirectedEdges();

    void printNode(ulonglong color);

public:

    AbstractGraph_6(RealWorld &realWorld,
    AbstractGraph &abG1, AbstractGraph_2 &abG2, AbstractGraph_3 &abG3, AbstractGraph_4 &abG4, AbstractGraph_5 &abG5) :
    rworld(realWorld), abGraph(abG1), abGraph2(abG2), abGraph3(abG3), abGraph4(abG4), abGraph5(abG5) {
    }

    void setGoalColor(ulonglong color);

    Node createNode(ulonglong color);

    bool isGoalReached(ulonglong color);

    AbstractNode& unrank(ulonglong rank);

    ulonglong getGoalColor();

    vector<AbstractNode> getAllAbstractNodes();

    void createAbstractGraph();

    ulonglong getStartColor();

    unordered_map<ulonglong, AbstractNode>& accessAbstractGraph();

    void printConnectedColors();
};


#endif //INC_658PROJECT_ABSTRACTGRAPH_6_H
