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

using namespace std;

/**
 *   ----> y
 *   |
 *   |
 *   v
 *   x
 */
class AbstractGraph {

    const int SECTOR_SIZE = 16;
    unordered_map<int, AbstractNode> colorAbstractNodeMap;

    int goalColor;

    void dfs(RealWorld &rworld, int x, int y, int sectorBoundaryX, int sectorBoundaryY,  int color);

    int dfsInASector(RealWorld &rworld, int sectorStartX, int sectorStartY, int startColor);

    void createUndirectedEdge(int color1, int color2);

    double findShortestDistanceToSectorCenter(int sectorBoundaryX, int sectorBoundaryY, int x, int y);

    void dfsToConnectAbstractNodes(RealWorld &rworld, int x, int y, vector<vector<bool>> &visited);

    void createAbstractGraphNodes(RealWorld &rworld);

    void connectAbstractNodesWithUndirectedEdges(RealWorld &rworld);

    /**
     * Helper Coordinates to find centroid
     */
     double minDistance;
     int nodesMarked;
     pair<int, int> centroid;

public:

    void createAbstractGraph(RealWorld &rworld);

    void printNode(int color);

    void setGoalColor(int color);

    int heuristic(int nodeColor);

    Node createNode(int color);

    bool isGoalReached(int color);

    AbstractNode unrank(ulonglong rank);

};


#endif //INC_658PROJECT_ABSTRACTGRAPH_H
