//
// Created by Debraj Ray on 2023-04-02.
//

#include "DataGeneratorForExperiments.h"
#include <iostream>
#include <filesystem>
#include <fstream>

#include <string>
#include <stdio.h>
#include <time.h>

void DataGeneratorForExperiments::populateFileNames() {
    std::string path = "/Users/debrajray/MyComputer/658/project/warcraft/wc3maps512-map";
    for (const auto & entry : filesystem::directory_iterator(path)) {
        if (entry.path().extension() == ".map") {
            fileNames.push_back(entry.path());
        }
    }
}

void DataGeneratorForExperiments::run() {
    populateFileNames();
    populateDataPoints();
    writeToFile();
}

void DataGeneratorForExperiments::populateDataPoints() {
    unique_ptr<unordered_set<ulonglong>> dummy = make_unique<unordered_set<ulonglong>>();
    bins.resize(13);
    int totalPaths = 0;
    unique_ptr<unordered_map<ulonglong, ulonglong>> pathRealWorld = make_unique<unordered_map<ulonglong, ulonglong>>();
    int skipTill = 2;

    //for(int j=skipTill; j <= fileNames.size()/2; ++j) {
    //    std::swap(fileNames[j], fileNames[fileNames.size() - j]);
    //}


    int i = 1;
    for(const string& fileName : fileNames) {
        if(fileName.empty()) continue;
        if (i++<skipTill) continue;
        cout<<"Map "<<fileName<<endl;
        int pathsCollected = 0;
        AStarSearch aStar;
        aStar.createAbstractGraph(fileName);
        vector<AbstractNode> abstractNodes;
        int countSkip1 = 0;
        for(int abLevel = 5; abLevel >= 2; --abLevel) {
        //for(int abLevel = 2; abLevel <= 5; ++abLevel) {
            aStar.copyAbstractNodesFromLevel(abLevel, abstractNodes);
            /**
            * Start and End nodes are picked from the centroid of the abstract nodes
            */
            bool skipping = false;
            for(int fromNodeIdx = 0; fromNodeIdx < abstractNodes.size() - 1; ++fromNodeIdx) {
                int countSkip2 = 0;
                if(skipping) {
                    if(countSkip1 > 100) {
                        break;
                    }
                    ++countSkip1;
                }
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
                    int pathLengthApprox;
                    if (aStar.checkPathExists(pathLengthApprox)) {
                        if(checkSkip(pathLengthApprox)) {
                            if(!skipping) {
                                cout<<"SKIP"<<endl;
                            }
                            if(countSkip2 > 20) {
                                break;
                            }
                            skipping = true;
                            ++countSkip2;
                            continue;
                        }
                        pathRealWorld->clear();
                        aStar.accessRealWorld()->setPathLength(0);
                        aStar.searchPathInRealWorldWithAstar(pathRealWorld, dummy, dataPoint, 0);
                        if(checkSkip(dataPoint.aStarPathLength)) {
                            if(!skipping) {
                                cout<<"SKIP"<<endl;
                            }
                            if(countSkip2 > 20) {
                                break;
                            }
                            skipping = true;
                            ++countSkip2;
                            continue;
                        }
                        skipping = false;
                        if (totalPaths >= 6000) {
                            /**
                             * Termination
                             */
                            cout<<"Done with data collection "<<endl;
                            return;
                        }
                        if(totalPaths % 100 == 0) {
                            cout<<"Completed "<<(totalPaths * 100 / 6000)<<"%, totalPaths: "<<totalPaths<<", Time: ";
                            printTime();
                            cout<<endl;
                        }
                        int binIdx = floor(dataPoint.aStarPathLength / 50);
                        ++bins[binIdx];
                        ++totalPaths;
                        ++pathsCollected;
                        if(pathsCollected > 2000) {
                            break;
                        }
                        cout<<"From: ("<<abstractNodes[fromNodeIdx].centroidRealNode.first<<", "<<abstractNodes[fromNodeIdx].centroidRealNode.second<<")  ";
                        cout<<"To: ("<<abstractNodes[toNodeIdx].centroidRealNode.first<<", "<<abstractNodes[toNodeIdx].centroidRealNode.second<<")  ";
                        cout<<"A* Path Length "<<dataPoint.aStarPathLength<<" Time "<<dataPoint.aStarExecTime<<endl;
                        if (!aStar.partialRefinementAStarSearch(10000, dataPoint)) {
                            throw std::runtime_error("PRA*(Inf) did not find a path, however PATH MUST EXIST");
                        }
                        //cout<<"PRA inf , path ratio "<<dataPoint.praStar_Inf_PathLength_ratio<<endl;
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
                        dataPoints.emplace_back(dataPoint);
                    }
                }
                if(pathsCollected > 1500) {
                    break;
                }
            }
            if(pathsCollected > 1500) {
                break;
            }
        }
    }
}

void DataGeneratorForExperiments::writeToFile() {
    cout<<"Writing data:"<<endl;
    ofstream myfile;
    myfile.open ("/Users/debrajray/MyComputer/658/project/wc3maps512_out_2.csv");
    for(const auto &datapoint: dataPoints) {
        myfile<<datapoint.aStarPathLength<<","<<datapoint.aStarExecTime<<","<<datapoint.praStar_Inf_PathLength<<","<<datapoint.praStar_Inf_ExecTime<<","<<datapoint.praStar_Inf_PathLength_ratio<<",";
        myfile<<datapoint.praStar_K16_PathLength<<","<<datapoint.praStar_K16_ExecTime<<","<<datapoint.praStar_K16_PathLength_ratio<<","<<datapoint.praStar_K8_PathLength<<","<<datapoint.praStar_K8_ExecTime<<","<<datapoint.praStar_K8_PathLength_ratio<<",";
        myfile<<datapoint.praStar_K4_PathLength<<","<<datapoint.praStar_K4_ExecTime<<","<<datapoint.praStar_K4_PathLength_ratio<<","<<datapoint.praStar_K2_PathLength<<","<<datapoint.praStar_K2_ExecTime<<","<<datapoint.praStar_K2_PathLength_ratio<<endl;
    }
    myfile.close();
}

void DataGeneratorForExperiments::printTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y/%m/%d  %X", &tstruct);
    cout<<buf;
}

bool DataGeneratorForExperiments::checkSkip(int pathLength) {
    int binIdx = floor(pathLength / 50);
    return (pathLength < 30 || pathLength >= 700) || (bins[binIdx] > 500);
}
