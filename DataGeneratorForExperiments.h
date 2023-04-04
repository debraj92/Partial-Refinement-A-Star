//
// Created by Debraj Ray on 2023-04-02.
//

#ifndef INC_658PROJECT_DATAGENERATORFOREXPERIMENTS_H
#define INC_658PROJECT_DATAGENERATORFOREXPERIMENTS_H

#include "DataPoint.h"
#include "AStarSearch.h"
#include <vector>

using namespace std;

class DataGeneratorForExperiments {

    vector<DataPoint> dataPoints;
    vector<string> fileNames;

    /**
     * There are 11 bins:
     * <50, <100, <150, 200, 250, 300, 350, 400, 450, 500, 550
     * 0      1    2      3    4    5    6    7    8    9    10
     * If path length = L, find floor(L / 50) -> i
     * ++bins[i]
     * if (i > 550) {skip}
     * if (bins[i] >= 20) {skip}
     */
    vector<int> bins;

    AStarSearch aStar;

    void populateFileNames();
    void populateDataPoints();

public:

    void test();
};


#endif //INC_658PROJECT_DATAGENERATORFOREXPERIMENTS_H
