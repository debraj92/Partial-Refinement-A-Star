//
// Created by Debraj Ray on 2023-03-25.
//

#ifndef INC_658PROJECT_REALWORLD_H
#define INC_658PROJECT_REALWORLD_H

#include <vector>
#include <iostream>
#include <string>
#include "Node.h"

using namespace std;
typedef unsigned long long ulonglong;

class RealWorld {


    vector<vector<int>> realMap;
    vector<vector<ulonglong>> mapColors;

    int goalX, goalY;
    int finalDestinationX, finalDestinationY;

    int startX, startY;
    int pathLength = 0;

public:
    const int MAX_SIZE = 512;

    void readMapFromFile(const string &fileName);

    void printMap();
    void printProvidedRealMap(vector<vector<int>> &givenMap);

    void printColors();

    vector<vector<ulonglong>>& getMapColors();
    vector<vector<int>>& getRealMap();

    ulonglong getRank(int x, int y);

    void unrank(ulonglong rank, int &x, int &y);

    void setGoalState(int x, int y);

    void setDestinationForHeuristics();

    double heuristic(int currentX, int currentY);

    double heuristic(ulonglong rank);

    Node createNode(int x, int y);

    bool isGoalReached(int x, int y);

    void getGoal(int &x, int &y);

    void setStart(int x, int y);

    void getStart(int &x, int &y) const;

    void setPathLength(int length);

    int getPathLength() const;

};


#endif //INC_658PROJECT_REALWORLD_H
