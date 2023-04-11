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

    unordered_map<ulonglong, AbstractNode> colorAbstractNodeMap;

    AbstractGraph &abGraph;
    RealWorld &rworld;

    ulonglong goalColor;

    void dfsToConnectAbstractNodes(const AbstractNode &abNode, ulonglong color, unordered_set<ulonglong> &visited);

    void createUndirectedEdge(ulonglong color1, ulonglong color2);

    void createAbstractGraphNodes();

    void createUndirectedEdges();

    void printNode(ulonglong color);

public:

    AbstractGraph_2(RealWorld &realWorld, AbstractGraph &abG1) : rworld(realWorld), abGraph(abG1) {

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
};


#endif //INC_658PROJECT_ABSTRACTGRAPH_2_H
