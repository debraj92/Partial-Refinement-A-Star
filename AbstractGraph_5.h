//
// Created by Debraj Ray on 2023-04-06.
//

#ifndef INC_658PROJECT_ABSTRACTGRAPH_5_H
#define INC_658PROJECT_ABSTRACTGRAPH_5_H

#include <vector>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include "AbstractNode.h"
#include "AbstractGraph_2.h"
#include "AbstractGraph_3.h"
#include "AbstractGraph_4.h"
#include "AbstractGraph.h"
#include "Abstraction.h"

using namespace std;

class AbstractGraph_5 : public Abstraction {

    unordered_map<int, AbstractNode> colorAbstractNodeMap;

    AbstractGraph_4 &abGraph4;
    AbstractGraph_3 &abGraph3;
    AbstractGraph_2 &abGraph2;
    AbstractGraph &abGraph;
    RealWorld &rworld;

    int goalColor;

    void dfsToConnectAbstractNodes(const AbstractNode &abNode, int abG5Color, unordered_set<int> &visited);

    void createUndirectedEdge(int color1, int color2);

    void createAbstractGraphNodes();

    void createUndirectedEdges();

    void printNode(int color);

public:

    AbstractGraph_5(RealWorld &realWorld,
                    AbstractGraph &abG1, AbstractGraph_2 &abG2, AbstractGraph_3 &abG3, AbstractGraph_4 &abG4) :
    rworld(realWorld), abGraph(abG1), abGraph2(abG2), abGraph3(abG3), abGraph4(abG4) {

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

    void printConnectedColors();
};


#endif //INC_658PROJECT_ABSTRACTGRAPH_5_H
