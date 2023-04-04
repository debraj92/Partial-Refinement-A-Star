//
// Created by Debraj Ray on 2023-04-02.
//

#include "DataGeneratorForExperiments.h"
#include <iostream>
#include <filesystem>

void DataGeneratorForExperiments::populateFileNames() {
    std::string path = "/Users/debrajray/MyComputer/658/project/baldurGate/";
    for (const auto & entry : filesystem::directory_iterator(path)) {
        if (entry.path().extension() == ".map") {
            fileNames.push_back(entry.path());
        }
    }
}

void DataGeneratorForExperiments::test() {
    populateFileNames();
    populateDataPoints();
}

void DataGeneratorForExperiments::populateDataPoints() {
    unique_ptr<unordered_set<ulonglong>> dummy = make_unique<unordered_set<ulonglong>>();
    bins.resize(11);
    int totalPaths = 0;
    for(const string& fileName : fileNames) {
        unique_ptr<unordered_map<ulonglong, ulonglong>> pathRealWorld = make_unique<unordered_map<ulonglong, ulonglong>>();
        unique_ptr<unordered_map<ulonglong, ulonglong>> pathAbstractWorld = make_unique<unordered_map<ulonglong, ulonglong>>();
        aStar.createAbstractGraph(fileName);
        auto abstractNodes = aStar.accessAbstractGraph()->getAllAbstractNodes();
        /**
         * Start and End nodes are picked from the centroid of the abstract nodes
         */
        for(int fromNodeIdx = 0; fromNodeIdx < abstractNodes.size() - 1; ++fromNodeIdx) {
            for(int toNodeIdx = fromNodeIdx + 1; toNodeIdx < abstractNodes.size(); ++toNodeIdx) {
                DataPoint dataPoint(
                        abstractNodes[fromNodeIdx].centroidRealNode.first,
                        abstractNodes[fromNodeIdx].centroidRealNode.second,
                        abstractNodes[toNodeIdx].centroidRealNode.first,
                        abstractNodes[toNodeIdx].centroidRealNode.second,
                        fileName
                        );
                aStar.initStartState(abstractNodes[fromNodeIdx].centroidRealNode.first, abstractNodes[fromNodeIdx].centroidRealNode.second);
                aStar.initGoalState(abstractNodes[toNodeIdx].centroidRealNode.first, abstractNodes[toNodeIdx].centroidRealNode.second);
                pathRealWorld->clear();
                aStar.accessRealWorld()->setPathLength(0);
                if (aStar.searchPathInRealWorldWithAstar(pathRealWorld, dummy, dataPoint)) {
                    if (dataPoint.aStarPathLength < 20 || dataPoint.aStarPathLength >= 550) {
                        continue;
                    }
                    int binIdx = floor(dataPoint.aStarPathLength / 50);
                    if(bins[binIdx] > 25) {
                        continue;
                    }
                    // 11,000 data points in total
                    // 5500 per game of 2 games
                    if (totalPaths >= 5500) {
                        /**
                         * Termination
                         */
                        return;
                    }
                    ++bins[binIdx];
                    ++totalPaths;
                    cout<<"A* path found, path length "<<dataPoint.aStarPathLength<<" Exec time "<<dataPoint.aStarExecTime<<endl;
                    if (!aStar.partialRefinementAStarSearch(0, dataPoint)) {
                        throw std::runtime_error("PRA*(Inf) did not find a path, however PATH MUST EXIST");
                    }
                    if (!aStar.partialRefinementAStarSearch(2, dataPoint)) {
                        throw std::runtime_error("PRA*(2) did not find a path, however PATH MUST EXIST");
                    }
                    if (!aStar.partialRefinementAStarSearch(4, dataPoint)) {
                        throw std::runtime_error("PRA*(4) did not find a path, however PATH MUST EXIST");
                    }
                    if (!aStar.partialRefinementAStarSearch(8, dataPoint)) {
                        throw std::runtime_error("PRA*(8) did not find a path, however PATH MUST EXIST");
                    }
                    if (!aStar.partialRefinementAStarSearch(16, dataPoint)) {
                        throw std::runtime_error("PRA*(16) did not find a path, however PATH MUST EXIST");
                    }
                }
            }
        }
        // TODO: Temporary
        break;
    }
}
