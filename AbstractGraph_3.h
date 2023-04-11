//
// Created by Debraj Ray on 2023-04-04.
//

#ifndef INC_658PROJECT_ABSTRACTGRAPH_3_H
#define INC_658PROJECT_ABSTRACTGRAPH_3_H

#include <vector>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include "AbstractNode.h"
#include "AbstractGraph_2.h"
#include "AbstractGraph.h"
#include "Abstraction.h"

using namespace std;

/**
 * This is a higher level abstraction on top of AbstractGraph_2.
 * It uses clique based abstraction where each clique can contain MAX 8 nodes
 */
class AbstractGraph_3 : public Abstraction {

    unordered_map<ulonglong, AbstractNode> colorAbstractNodeMap;

    const int MAX_EDGE_LENGTH = 4;

    AbstractGraph_2 &abGraph2;
    AbstractGraph &abGraph;
    RealWorld &rworld;

    ulonglong goalColor;

    void dfsToConnectAbstractNodes(const AbstractNode &abNode, ulonglong abG3Color, unordered_set<ulonglong> &visited);

    void createUndirectedEdge(ulonglong color1, ulonglong color2);

    void createAbstractGraphNodes();
    void createUndirectedEdges();

    void printNode(ulonglong color);

public:

    AbstractGraph_3(RealWorld &realWorld, AbstractGraph &abG1, AbstractGraph_2 &abG2) : rworld(realWorld), abGraph(abG1), abGraph2(abG2) {

    }

    void setGoalColor(ulonglong color);

    Node createNode(ulonglong color);

    bool isGoalReached(ulonglong color);

    // TODO: Risk check for return by reference
    AbstractNode& unrank(ulonglong rank);

    ulonglong getGoalColor();

    vector<AbstractNode> getAllAbstractNodes();

    void createAbstractGraph();

    ulonglong getStartColor();

    unordered_map<ulonglong, AbstractNode>& accessAbstractGraph();

    void printConnectedColors();
};


#endif //INC_658PROJECT_ABSTRACTGRAPH_3_H
