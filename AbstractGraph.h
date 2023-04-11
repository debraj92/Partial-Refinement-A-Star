//
// Created by Debraj Ray on 2023-03-25.
//

#ifndef INC_658PROJECT_ABSTRACTGRAPH_H
#define INC_658PROJECT_ABSTRACTGRAPH_H

#include <vector>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include "AbstractNode.h"
#include "RealWorld.h"
#include "Abstraction.h"

using namespace std;

/**
 *   ----> y
 *   |
 *   |
 *   v
 *   x
 */
class AbstractGraph : public Abstraction {

    const int SECTOR_SIZE = 2;
    unordered_map<ulonglong, AbstractNode> colorAbstractNodeMap;

    RealWorld &rworld;

    ulonglong goalColor;

    void dfs(int x, int y, int sectorBoundaryX, int sectorBoundaryY,  ulonglong color);

    ulonglong dfsInASector(int sectorStartX, int sectorStartY, ulonglong startColor);

    void createUndirectedEdge(ulonglong color1, ulonglong color2);

    double findShortestDistanceToSectorCenter(int sectorBoundaryX, int sectorBoundaryY, int x, int y);

    void dfsToConnectAbstractNodes(int x, int y, const ulonglong parentColor, vector<vector<bool>> &visited);

    void createAbstractGraphNodes();

    void connectAbstractNodesWithUndirectedEdges();

    int nodesMarked;
    int sumX, sumY;
    /**
     * Helper Coordinates to find centroid
     */
     double minDistanceCentroid;
     pair<int, int> centroid;

public:

    AbstractGraph(RealWorld &realWorld) : rworld(realWorld) {

    }

    void createAbstractGraph();

    void printNode(ulonglong color);

    void setGoalColor(ulonglong color);

    Node createNode(ulonglong color);

    bool isGoalReached(ulonglong color);

    AbstractNode& unrank(ulonglong rank);

    ulonglong getGoalColor();

    vector<AbstractNode> getAllAbstractNodes();

    unordered_map<ulonglong, AbstractNode>& accessAbstractGraph();

    ulonglong getStartColor();

};


#endif //INC_658PROJECT_ABSTRACTGRAPH_H
