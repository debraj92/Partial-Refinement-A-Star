//
// Created by Debraj Ray on 2023-03-27.
//

#ifndef INC_658PROJECT_ASTARSEARCH_H
#define INC_658PROJECT_ASTARSEARCH_H

#include "AbstractGraph.h"
#include "AbstractGraph_2.h"
#include "AbstractGraph_3.h"
#include "AbstractGraph_4.h"
#include "AbstractGraph_5.h"
#include "RealWorld.h"
#include "AStarOpenList.h"
#include "DataPoint.h"
#include "Abstraction.h"

class AStarSearch {

    /**
     * 4 layers of abstraction constructed over the real world
     */
    unique_ptr<AbstractGraph_5> abstractGraph5;
    unique_ptr<AbstractGraph_4> abstractGraph4;
    unique_ptr<AbstractGraph_3> abstractGraph3;
    unique_ptr<AbstractGraph_2> abstractGraph2;
    unique_ptr<AbstractGraph> abstractGraph;
    unique_ptr<RealWorld> realWorld;

    void insertIntoOpenList(AStarOpenList &openList,
                            unique_ptr<unordered_set<ulonglong>> &closedList,
                            unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent,
                            ulonglong childRank,
                            Node &parentNode,
                            const std::function<double(ulonglong)>& hCost,
                            double gCost);

    void finalizeNodeLinks(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, unique_ptr<unordered_set<ulonglong>> &closedList);

    int reverseNodeLinks(ulonglong current, unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent);

    void copyRealWorld(vector<vector<int>> &copied);

public:

    void createAbstractGraph(const string &fileName);
    void createRealWorld(const string &fileName);

    void initStartState(int startX, int startY);
    void initGoalState(int goalX, int goalY);

    /**
     * For PRA*, provide non-empty abstractParentNodes. For A* this should be empty.
     */
    bool searchPathInRealWorldWithAstar(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent,
                                        const unique_ptr<unordered_set<ulonglong>> &abstractParentNodes,
                                        DataPoint &dataPoint);

    /**
     * Search a path through an abstraction level. It uses constrained A* search that expands nodes whose parent node
     * is part of the abstract path in the next higher level of abstraction
     */
    bool searchPathInAbstractGraphWithAstar(unique_ptr<unordered_map<ulonglong,
                                            ulonglong>> &childParent,
                                            unique_ptr<unordered_set<ulonglong>> &abstractParentNodes,
                                            int abstractionLevel,
                                            ulonglong parentGoalColor);

    bool searchAndTruncatePathInAbstractWorld (int K, unique_ptr<unordered_set<ulonglong>> &abstractParentNodes, ulonglong &parentGoalColor);

    int getStartColorOfAbstraction(int abstractionLevel);

    int getGoalColorOfAbstraction(int abstractionLevel);

    bool partialRefinementAStarSearch(int K, DataPoint &dataPoint);

    void printRealPath(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank, ulonglong destinationRank);

    void printAbstractPath(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank, ulonglong destinationRank, Abstraction *abstraction);

    unique_ptr<AbstractGraph>& accessAbstractGraph();
    unique_ptr<RealWorld>& accessRealWorld();

    void printPathNodes(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank, ulonglong destinationRank);

    struct InitialState {
        int startX;
        int startY;
        int goalX;
        int goalY;

        void init (int sx, int sy, int gx, int gy) {
            startX = sx;
            startY = sy;
            goalX = gx;
            goalY = gy;
        }
    };

    void storeInitialState(InitialState &initState);
    void restoreInitialState(InitialState &initState);


};


#endif //INC_658PROJECT_ASTARSEARCH_H
