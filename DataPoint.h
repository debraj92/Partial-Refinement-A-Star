//
// Created by Debraj Ray on 2023-04-02.
//

#ifndef INC_658PROJECT_DATAPOINT_H
#define INC_658PROJECT_DATAPOINT_H

#include <string>
#include <utility>

using namespace std;
class DataPoint {

public:

    DataPoint(int startX, int startY, int goalX, int goalY, string mapName) :
    startX(startX), startY(startY), goalX(goalX), goalY(goalY), mapName(std::move(mapName)){

    }

    // Only for dummy creation
    DataPoint() {}

    string mapName;
    int startX;
    int startY;
    int goalX;
    int goalY;

    int aStarPathLength;
    int aStarExecTime;

    int praStar_Inf_PathLength;
    int praStar_Inf_ExecTime;
    double praStar_Inf_PathLength_ratio;

    int praStar_K2_PathLength;
    int praStar_K2_ExecTime;
    double praStar_K2_PathLength_ratio;

    int praStar_K4_PathLength;
    int praStar_K4_ExecTime;
    double praStar_K4_PathLength_ratio;

    int praStar_K8_PathLength;
    int praStar_K8_ExecTime;
    double praStar_K8_PathLength_ratio;

    int praStar_K16_PathLength;
    int praStar_K16_ExecTime;
    double praStar_K16_PathLength_ratio;

    void setPraStarKData (int K, int pathLength, int execTime) {
        switch (K) {
            case 2:
                praStar_K2_PathLength = pathLength;
                praStar_K2_ExecTime = execTime;
                praStar_K2_PathLength_ratio = (double) praStar_K2_PathLength / aStarPathLength;
                break;
            case 4:
                praStar_K4_PathLength = pathLength;
                praStar_K4_ExecTime = execTime;
                praStar_K4_PathLength_ratio = (double) praStar_K4_PathLength / aStarPathLength;
                break;
            case 8:
                praStar_K8_PathLength = pathLength;
                praStar_K8_ExecTime = execTime;
                praStar_K8_PathLength_ratio = (double) praStar_K8_PathLength / aStarPathLength;
                break;
            case 16:
                praStar_K16_PathLength = pathLength;
                praStar_K16_ExecTime = execTime;
                praStar_K16_PathLength_ratio = (double) praStar_K16_PathLength / aStarPathLength;
                break;
            default:
                praStar_Inf_PathLength = pathLength;
                praStar_Inf_ExecTime = execTime;
                praStar_Inf_PathLength_ratio = (double) praStar_Inf_PathLength / aStarPathLength;
                break;
        }
    }

};


#endif //INC_658PROJECT_DATAPOINT_H
