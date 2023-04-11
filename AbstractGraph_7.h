//
// Created by Debraj Ray on 2023-04-10.
//

#ifndef INC_658PROJECT_ABSTRACTGRAPH_7_H
#define INC_658PROJECT_ABSTRACTGRAPH_7_H

#include <vector>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include "AbstractNode.h"
#include "AbstractGraph_2.h"
#include "AbstractGraph_3.h"
#include "AbstractGraph_4.h"
#include "AbstractGraph_5.h"
#include "AbstractGraph_6.h"
#include "AbstractGraph.h"
#include "Abstraction.h"

using namespace std;

class AbstractGraph_7 : public Abstraction {

    const int MAX_EDGE_LENGTH = 64;

    unordered_map<ulonglong, AbstractNode> colorAbstractNodeMap;

    AbstractGraph_6 &abGraph6;
    AbstractGraph_5 &abGraph5;
    AbstractGraph_4 &abGraph4;
    AbstractGraph_3 &abGraph3;
    AbstractGraph_2 &abGraph2;
    AbstractGraph &abGraph;
    RealWorld &rworld;

    ulonglong goalColor;

    void dfsToConnectAbstractNodes(const AbstractNode &abNode, ulonglong abG7Color, unordered_set<ulonglong> &visited);

    void createUndirectedEdge(ulonglong color1, ulonglong color2);

    void createAbstractGraphNodes();

    void createUndirectedEdges();

    void printNode(int color);

public:

    AbstractGraph_7(RealWorld &realWorld,
    AbstractGraph &abG1, AbstractGraph_2 &abG2, AbstractGraph_3 &abG3, AbstractGraph_4 &abG4, AbstractGraph_5 &abG5, AbstractGraph_6 &abG6) :
    rworld(realWorld), abGraph(abG1), abGraph2(abG2), abGraph3(abG3), abGraph4(abG4), abGraph5(abG5), abGraph6(abG6) {
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


#endif //INC_658PROJECT_ABSTRACTGRAPH_7_H
