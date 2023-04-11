//
// Created by Debraj Ray on 2023-03-25.
//

#include "RealWorld.h"
#include <fstream>
#include <cmath>

void RealWorld::readMapFromFile(const string &fileName) {
    std::ifstream file(fileName);
    if (file.is_open()) {
        std::string line;
        /**
         * discard 4 lines
         * type, height, width, map
         */
         for(int i=0; i<4; ++i) {
             std::getline(file, line);
         }
         /**
          * Now we have to read 512 X 512 data
          */
         line = "";
         if(realMap.capacity() == 0) {
             realMap.resize(MAX_SIZE);
             mapColors.resize(MAX_SIZE);
         }
        int row = 0, col = 0;
        while (std::getline(file, line)) {
            if(realMap[row].capacity() == 0) {
                realMap[row].resize(MAX_SIZE);
                mapColors[row].resize(MAX_SIZE);
            }
            for(char x: line) {
                if(x == '.') {
                    realMap[row][col] = 0;
                } else {
                    realMap[row][col] = 1;
                }
                mapColors[row][col] = -1;
                ++col;
            }
            ++row;
            col = 0;
        }
        file.close();
    } else {
        throw std::runtime_error("Could not open file: " + fileName);
    }
}

void RealWorld::printMap() {
    for(int i=0; i<MAX_SIZE; ++i) {
        for(int j=0; j<MAX_SIZE; ++j) {
            cout<<realMap[i][j];
        }
        cout<<endl;
    }
}

void RealWorld::printColors() {
    for(int i=0; i<MAX_SIZE; ++i) {
        for(int j=0; j<MAX_SIZE; ++j) {
            if(mapColors[i][j] < 0) {
                cout<<".   ";
            } else {
                cout<<mapColors[i][j];
                if (mapColors[i][j] < 10) {
                    cout<<"   ";
                } else if (mapColors[i][j] < 100) {
                    cout<<"  ";
                } else if (mapColors[i][j] >= 100) {
                    cout<<" ";
                }
            }
        }
        cout<<endl;
    }
}

void RealWorld::printProvidedRealMap(vector<vector<int>> &givenMap) {
    for(int i=0; i<MAX_SIZE; ++i) {
        for(int j=0; j<MAX_SIZE; ++j) {
            cout<<givenMap[i][j];
        }
        cout<<endl;
    }
}

vector<vector<ulonglong>> &RealWorld::getMapColors() {
    return mapColors;
}

vector<vector<int>> &RealWorld::getRealMap() {
    return realMap;
}

/**
 * Assumes x and y < 1000
 * In this case x and y < 512
 */
ulonglong RealWorld::getRank(int x, int y) {
    return x * 1000 + y;
}

double RealWorld::heuristic(int currentX, int currentY) {
    return sqrt(pow(currentX - finalDestinationX, 2) + pow(currentY - finalDestinationY, 2));
}

void RealWorld::setGoalState(int x, int y) {
    goalX = x;
    goalY = y;
}

Node RealWorld::createNode(int x, int y) {
    return {getRank(x, y)};
}

bool RealWorld::isGoalReached(int x, int y) {
    return x == goalX && y == goalY;
}

void RealWorld::unrank(ulonglong rank, int &x, int &y) {
    y = (int)(rank % 1000);
    x = (int)(rank / 1000);
}

double RealWorld::heuristic(ulonglong rank) {
    int x, y;
    unrank(rank, x, y);
    return heuristic(x, y);
}

void RealWorld::getGoal(int &x, int &y) {
    x = goalX;
    y = goalY;
}

void RealWorld::setStart(int x, int y) {
    startX = x;
    startY = y;
}

void RealWorld::getStart(int &x, int &y) const {
    x = startX;
    y = startY;
}

void RealWorld::setPathLength(int length) {
    pathLength = length;
}

int RealWorld::getPathLength() const {
    return pathLength;
}

void RealWorld::setDestinationForHeuristics() {
    finalDestinationX = goalX;
    finalDestinationY = goalY;
}

