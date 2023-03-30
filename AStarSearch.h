//
// Created by Debraj Ray on 2023-03-27.
//

#ifndef INC_658PROJECT_ASTARSEARCH_H
#define INC_658PROJECT_ASTARSEARCH_H

#include "AbstractGraph.h"
#include "RealWorld.h"
#include "AStarOpenList.h"

class AStarSearch {

    unique_ptr<AbstractGraph> abstractGraph;
    unique_ptr<RealWorld> realWorld;

    void insertIntoOpenList(AStarOpenList &openList,
                            unique_ptr<unordered_set<ulonglong>> &closedList,
                            unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent,
                            ulonglong childRank,
                            Node &parentNode);

    void finalizeNodeLinks(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, unique_ptr<unordered_set<ulonglong>> &closedList);

    int reverseNodeLinks(ulonglong current, unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent);

    void copyRealWorld(vector<vector<int>> &copied);

    bool constrainedAStarSearch(int startX, int startY, int K);

public:

    void createAbstractGraph(const string &fileName, int goalX, int goalY);
    void createRealWorld(const string &fileName, int goalX, int goalY);

    bool searchPathInRealWorldWithAstar(int startX, int startY);

    bool searchPathInAbstractGraphWithAstar(int startX, int startY);

    void printRealPath(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank);

    void printAbstractPath(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank);

};


#endif //INC_658PROJECT_ASTARSEARCH_H
