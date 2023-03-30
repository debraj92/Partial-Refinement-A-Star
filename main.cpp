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
    realWorld.printMap();
    AbstractGraph abstractGraph;
    abstractGraph.createAbstractGraph(realWorld);
    realWorld.printColors();
    //abstractGraph.printConnectedColors();
    */

    AStarSearch aStar;
    aStar.createAbstractGraph(fName, 303, 403);
    aStar.searchPathInRealWorldWithAstar(175, 56);
    //aStar.searchPathInAbstractGraphWithAstar(175, 56);

    return 0;
}
