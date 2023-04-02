#include <iostream>
#include "RealWorld.h"
#include "AbstractGraph.h"
#include "AStarSearch.h"
#include <string>

int main() {
    string fName = "/Users/debrajray/MyComputer/658/project/wc3maps512-map/selected maps/hillsofglory.map";
    /*
    RealWorld realWorld;
    realWorld.readMapFromFile(fName);
    //realWorld.printMap();
    AbstractGraph abstractGraph;
    abstractGraph.createAbstractGraph(realWorld);
    realWorld.printColors();
    //abstractGraph.printConnectedColors();
    */

    unique_ptr<unordered_map<ulonglong, ulonglong>> pathRealWorld = make_unique<unordered_map<ulonglong, ulonglong>>();
    unique_ptr<unordered_map<ulonglong, ulonglong>> pathAbstractWorld = make_unique<unordered_map<ulonglong, ulonglong>>();
    unique_ptr<unordered_set<ulonglong>> dummy = make_unique<unordered_set<ulonglong>>();

    AStarSearch aStar;
    aStar.createAbstractGraph(fName, 52, 380, 393, 73);
    //aStar.searchPathInAbstractGraphWithAstar(pathAbstractWorld);
    //aStar.searchPathInRealWorldWithAstar(pathRealWorld, dummy);

    //aStar.partialRefinementAStarSearch(0);
    aStar.partialRefinementAStarSearch(5);

    return 0;
}
