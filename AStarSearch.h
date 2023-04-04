//
// Created by Debraj Ray on 2023-03-27.
//

#ifndef INC_658PROJECT_ASTARSEARCH_H
#define INC_658PROJECT_ASTARSEARCH_H

#include "AbstractGraph.h"
#include "RealWorld.h"
#include "AStarOpenList.h"
#include "DataPoint.h"

class AStarSearch {

    unique_ptr<AbstractGraph> abstractGraph;
    unique_ptr<RealWorld> realWorld;

    void insertIntoOpenList(AStarOpenList &openList,
                            unique_ptr<unordered_set<ulonglong>> &closedList,
                            unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent,
                            ulonglong childRank,
                            Node &parentNode,
                            const std::function<double(ulonglong)>& heuristic);

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

    bool searchPathInAbstractGraphWithAstar(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent);

    bool partialRefinementAStarSearch(int K, DataPoint &dataPoint);

    void printRealPath(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank, ulonglong destinationRank);

    void printAbstractPath(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank);

    unique_ptr<AbstractGraph>& accessAbstractGraph();
    unique_ptr<RealWorld>& accessRealWorld();

    void printPathNodes(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank, ulonglong destinationRank);

    struct InitialState {
        int startX;
        int startY;
        int goalX;
        int goalY;
        int goalColor;

        void init (int sx, int sy, int gx, int gy, int gcolor) {
            startX = sx;
            startY = sy;
            goalX = gx;
            goalY = gy;
            goalColor = gcolor;
        }
    };

    void storeInitialState(InitialState &initState);
    void restoreInitialState(InitialState &initState);


};


#endif //INC_658PROJECT_ASTARSEARCH_H
