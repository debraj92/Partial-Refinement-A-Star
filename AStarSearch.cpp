//
// Created by Debraj Ray on 2023-03-27.
//

#include "AStarSearch.h"
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

void AStarSearch::createAbstractGraph(const string &fileName) {
    cout<<"Creating abstract graphs"<<endl;
    createRealWorld(fileName);
    abstractGraph = make_unique<AbstractGraph>(*realWorld);
    abstractGraph->createAbstractGraph();

    abstractGraph2 = make_unique<AbstractGraph_2>(*realWorld, *abstractGraph);
    abstractGraph2->createAbstractGraph();

    abstractGraph3 = make_unique<AbstractGraph_3>(*realWorld, *abstractGraph, *abstractGraph2);
    abstractGraph3->createAbstractGraph();

    abstractGraph4 = make_unique<AbstractGraph_4>(*realWorld, *abstractGraph, *abstractGraph2, *abstractGraph3);
    abstractGraph4->createAbstractGraph();

    abstractGraph5 = make_unique<AbstractGraph_5>(*realWorld, *abstractGraph, *abstractGraph2, *abstractGraph3, *abstractGraph4);
    abstractGraph5->createAbstractGraph();

    abstractGraph6 = make_unique<AbstractGraph_6>(*realWorld, *abstractGraph, *abstractGraph2, *abstractGraph3, *abstractGraph4, *abstractGraph5);
    abstractGraph6->createAbstractGraph();

    abstractGraph7 = make_unique<AbstractGraph_7>(*realWorld, *abstractGraph, *abstractGraph2, *abstractGraph3, *abstractGraph4, *abstractGraph5, *abstractGraph6);
    abstractGraph7->createAbstractGraph();
}

void AStarSearch::initStartState(int startX, int startY) {
    realWorld->setStart(startX, startY);
}

void AStarSearch::initGoalState(int goalX, int goalY) {
    realWorld->setGoalState(goalX, goalY);
    realWorld->setDestinationForHeuristics();
    abstractGraph->setGoal(goalX, goalY);
    abstractGraph2->setGoal(goalX, goalY);
    abstractGraph3->setGoal(goalX, goalY);
    abstractGraph4->setGoal(goalX, goalY);
    abstractGraph5->setGoal(goalX, goalY);
    abstractGraph6->setGoal(goalX, goalY);
    abstractGraph7->setGoal(goalX, goalY);

    abstractGraph->setGoalColor(realWorld->getMapColors()[goalX][goalY]);
    abstractGraph2->setGoalColor(
            abstractGraph->unrank(abstractGraph->getGoalColor()).abstractionColor
    );
    abstractGraph3->setGoalColor(
            abstractGraph2->unrank(abstractGraph2->getGoalColor()).abstractionColor
    );
    abstractGraph4->setGoalColor(
            abstractGraph3->unrank(abstractGraph3->getGoalColor()).abstractionColor
    );
    abstractGraph5->setGoalColor(
            abstractGraph4->unrank(abstractGraph4->getGoalColor()).abstractionColor
    );
    abstractGraph6->setGoalColor(
            abstractGraph5->unrank(abstractGraph5->getGoalColor()).abstractionColor
    );
    abstractGraph7->setGoalColor(
            abstractGraph6->unrank(abstractGraph6->getGoalColor()).abstractionColor
    );
}

void AStarSearch::createRealWorld(const string &fileName) {
    realWorld = make_unique<RealWorld>();
    realWorld->readMapFromFile(fileName);
}

/**
 * If abstractParentNodes is empty then this function is plain A* search.
 * Otherwise this is a constrained A* search as required in PRA*.
 */
bool AStarSearch::searchPathInRealWorldWithAstar(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent,
                                                 unique_ptr<unordered_set<ulonglong>> &abstractParentNodes,
                                                 DataPoint &dataPoint,
                                                 ulonglong parentGoalColor) {
    /**
     * Generalizes the heuristic function with a common interface
     */
    auto heuristicFunc = [this](ulonglong rank) {
        return realWorld->heuristic(rank);
    };
    unique_ptr<unordered_set<ulonglong>> closedList = make_unique<unordered_set<ulonglong>>();
    auto t1 = high_resolution_clock::now();
    AStarOpenList openList;
    int startX, startY;
    realWorld->getStart(startX, startY);
    /// Starting point should be passable coordinate. Or non-obstacle.
    if (realWorld->getRealMap()[startX][startY] != 0) {
        cout<<"Starting point should be passable (non-obstacle) coordinate."<<endl;
        return false;
    }
    auto root = realWorld->createNode(startX, startY);
    auto h = realWorld->heuristic(startX, startY);
    root.calculateF(0, h);
    openList.insert(root);
    if (childParent->contains(root.rank)) {
        (*childParent)[root.rank] = root.rank;
    } else {
        childParent->insert({root.rank, root.rank});
    }
    int x, y;
    bool isPathFound = false;
    int goalX, goalY;
    /**
     * Moving Goal
     */
    realWorld->getGoal(goalX, goalY);
    /**
     * Either this is the actual goal, in which case (goalX, goalY) should be passable
     * Or this is an intermediate goal of PRA*, in which case this goal color should not match the final goal color.
     */
     if (realWorld->getRealMap()[goalX][goalY] != 0 && realWorld->getMapColors()[goalX][goalY] == abstractGraph->getGoalColor()) {
         cout<<"Goal location should either be passable or goal must be intermediate"<<endl;
         return false;
     }
    // True Goal in PRA*
    ulonglong finalGoalColor = abstractGraph->getGoalColor();
    double sqrt2 = sqrt(2);
    bool isLastMile = (!parentGoalColor) || finalGoalColor == parentGoalColor;
    while(!openList.isEmpty()) {
        Node nextNode = openList.removeMinimum();
        closedList->insert(nextNode.rank);

        // un-rank
        realWorld->unrank(nextNode.rank, x, y);

        /**
         * Additional termination condition for PRA*:
         * If abstract target node is destination node
         *  - RealWorld search should go till actual destination x, y
         *  - This case is similar to plain A* search
         * Else
         *  - RealWorld search should terminate at any x,y in destination node
         */

        if (realWorld->isGoalReached(x, y) || (!isLastMile && realWorld->getRealMap()[x][y] == parentGoalColor)) {
            auto solutionLength = reverseNodeLinks(nextNode.rank, childParent);
            //cout<<"Path Length Real World "<<solutionLength<<endl;
            //printRealPath(childParent, root.rank, nextNode.rank);
            isPathFound = true;
            dataPoint.aStarPathLength = solutionLength;
            if (!abstractParentNodes->empty()) {
                /**
                 * PRA*
                 */
                /// We update start point for PRA*
                realWorld->setStart(x, y);
                // We aggregate path length for PRA*.
                realWorld->setPathLength(solutionLength + realWorld->getPathLength());
            } else {
                realWorld->setPathLength(solutionLength);
            }
            break;
        }

        // generate moves
        /**
         * For PRA*, child nodes must be of a color that is in the abstract path
         * For A*, there is no restriction.
         * A* is the case when abstractParentNodes is empty
         */
        // up
        if (x > 0 && realWorld->getRealMap()[x-1][y] == 0 &&
        (abstractParentNodes->empty() || abstractParentNodes->contains(realWorld->getMapColors()[x-1][y]))
        ) {
            ulonglong childRank = realWorld->getRank(x-1, y);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc, 1);
        }
        // down
        if (x < realWorld->MAX_SIZE - 1 && realWorld->getRealMap()[x+1][y] == 0 &&
                (abstractParentNodes->empty() || abstractParentNodes->contains(realWorld->getMapColors()[x+1][y]))
        ) {
            ulonglong childRank = realWorld->getRank(x+1, y);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc, 1);
        }
        // left
        if (y > 0 && realWorld->getRealMap()[x][y-1] == 0 &&
                (abstractParentNodes->empty() || abstractParentNodes->contains(realWorld->getMapColors()[x][y-1]))
        ) {
            ulonglong childRank = realWorld->getRank(x, y-1);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc, 1);
        }
        // right
        if (y < realWorld->MAX_SIZE - 1 && realWorld->getRealMap()[x][y+1] == 0 &&
                (abstractParentNodes->empty() || abstractParentNodes->contains(realWorld->getMapColors()[x][y+1]))
        ) {
            ulonglong childRank = realWorld->getRank(x, y+1);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc, 1);
        }

        /// diagonal moves
        if (x > 0 && y > 0 && realWorld->getRealMap()[x-1][y-1] == 0 &&
        (abstractParentNodes->empty() || abstractParentNodes->contains(realWorld->getMapColors()[x-1][y-1]))
        ) {
            ulonglong childRank = realWorld->getRank(x-1, y-1);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc, sqrt2);
        }

        if (x > 0 && y < realWorld->MAX_SIZE - 1 && realWorld->getRealMap()[x-1][y+1] == 0 &&
            (abstractParentNodes->empty() || abstractParentNodes->contains(realWorld->getMapColors()[x-1][y+1]))
            ) {
            ulonglong childRank = realWorld->getRank(x-1, y+1);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc, sqrt2);
        }

        if (x < realWorld->MAX_SIZE - 1 && y > 0 && realWorld->getRealMap()[x+1][y-1] == 0 &&
            (abstractParentNodes->empty() || abstractParentNodes->contains(realWorld->getMapColors()[x+1][y-1]))
            ) {
            ulonglong childRank = realWorld->getRank(x+1, y-1);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc, sqrt2);
        }

        if (x < realWorld->MAX_SIZE - 1 && y < realWorld->MAX_SIZE - 1 && realWorld->getRealMap()[x+1][y+1] == 0 &&
            (abstractParentNodes->empty() || abstractParentNodes->contains(realWorld->getMapColors()[x+1][y+1]))
            ) {
            ulonglong childRank = realWorld->getRank(x+1, y+1);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc, sqrt2);
        }
    }
    if (!isPathFound) {
        cout<<"Path Not Found"<<endl;
    }

    auto t2 = high_resolution_clock::now();
    auto time_in_ms = duration_cast<chrono::milliseconds>(t2 - t1);
    //cout<<"Total execution time "<<time_in_ms.count()<<endl;
    dataPoint.aStarExecTime = (int) time_in_ms.count();

    return isPathFound;
}

void
AStarSearch::insertIntoOpenList(AStarOpenList &openList,
                                unique_ptr<unordered_set<ulonglong>> &closedList,
                                unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent,
                                ulonglong childRank,
                                Node &parentNode,
                                const std::function<double(ulonglong)>& hCost,
                                double gCost) {

    // do not insert in open list if node already exists in closed
    if (closedList->contains(childRank)) {
        return;
    }
    Node childNode(childRank);
    if (!openList.isPresent(childNode)) {
        /**
         * New node, never visited. Just insert in open.
         */
        childNode.calculateF(parentNode.g + gCost, hCost(childRank));
        openList.insert(childNode);
        // child -> parent
        if (childParent->find(childNode.rank) != childParent->end()) {
            childParent->find(childNode.rank)->second = parentNode.rank;
        } else {
            childParent->insert({childNode.rank, parentNode.rank});
        }
    } else {
        /**
         * Node exists in open. Insert if better path found
         */
        if (openList.updateIfBetterPath(childNode, parentNode.g + gCost)) {
            childParent->find(childNode.rank)->second = parentNode.rank;
        }
    }
}

void AStarSearch::finalizeNodeLinks(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent,
                                    unique_ptr<unordered_set<ulonglong>> &closedList) {

    auto childParentIterator = childParent->begin();
    while(childParentIterator != childParent->end()) {
        auto child = childParentIterator->first;
        auto parent = childParentIterator->second;
        if (closedList->find(child) == closedList->end() || closedList->find(parent) == closedList->end()) {
            childParentIterator++;
            childParent->erase(child);
        } else {
            childParentIterator++;
        }
    }
}

int AStarSearch::reverseNodeLinks(ulonglong current, unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent) {

    auto parent = childParent->find(current)->second;
    auto grandParent = childParent->find(parent)->second;
    int count = 0;
    while (parent != grandParent) {
        (*childParent)[parent] = current;
        current = parent;
        parent = grandParent;
        grandParent = childParent->find(parent)->second;
        count++;
    }
    (*childParent)[parent] = current;
    return count+1;
}

void AStarSearch::printRealPath(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank, ulonglong destinationRank) {
    vector<vector<int>> copied(realWorld->MAX_SIZE);
    copyRealWorld(copied);
    int x, y;
    auto current = childParent->find(rootRank)->first;
    realWorld->unrank(current, x, y);
    copied[x][y] = 9;
    while(current != destinationRank) {
        current = childParent->find(current)->second;
        realWorld->unrank(current, x, y);
        copied[x][y] = 9;
    }
    realWorld->printProvidedRealMap(copied);
}


void AStarSearch::printAbstractPath(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank,
                                    ulonglong destinationRank,
                                    Abstraction *abstraction) {
    auto current = childParent->find(rootRank)->first;
    cout<<"Abstract Path Colours "<<endl;
    cout<<current<<" ";
    while(current != destinationRank) {
        current = childParent->find(current)->second;
        cout<<current<<" ";
    }
    cout<<endl;


    //// Prints Abstract path in real world
    vector<vector<int>> copied(realWorld->MAX_SIZE);
    copyRealWorld(copied);
    current = childParent->find(rootRank)->first;
    auto abstractNode = abstraction->unrank(current);
    copied[(int)round(abstractNode.representationCenter.first)][(int)round(abstractNode.representationCenter.second)] = 9;
    while(current != destinationRank) {
        current = childParent->find(current)->second;
        abstractNode = abstraction->unrank(current);
        copied[(int)round(abstractNode.representationCenter.first)][(int)round(abstractNode.representationCenter.second)] = 9;
    }
    realWorld->printProvidedRealMap(copied);
}

void AStarSearch::copyRealWorld(vector<vector<int>> &copied) {
    for(int row = 0; row < realWorld->MAX_SIZE; ++row) {
        copied[row].reserve(realWorld->MAX_SIZE);
        for(int col = 0; col < realWorld->MAX_SIZE; ++col) {
            copied[row][col] = realWorld->getRealMap()[row][col];
        }
    }
}


///////// AStar search in Abstract World //////////////////
bool AStarSearch::searchPathInAbstractGraphWithAstar(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent,
                                                     unique_ptr<unordered_set<ulonglong>> &abstractParentNodes,
                                                     int abstractionLevel,
                                                     ulonglong parentGoalColor,
                                                     ulonglong &startColor) {

    Abstraction *abstraction;
    switch (abstractionLevel) {
        case 1:
            abstraction = abstractGraph.get();
            break;
        case 2:
            abstraction = abstractGraph2.get();
            break;
        case 3:
            abstraction = abstractGraph3.get();
            break;
        case 4:
            abstraction = abstractGraph4.get();
            break;
        case 5:
            abstraction = abstractGraph5.get();
            break;
        case 6:
            abstraction = abstractGraph6.get();
            break;
        case 7:
            abstraction = abstractGraph7.get();
            break;
    }

    unique_ptr<unordered_set<ulonglong>> closedList = make_unique<unordered_set<ulonglong>>();
    startColor = abstraction->getStartColor();

    if(startColor == -1) {
        cout<<"Abstract Path Not Found"<<endl;
        return false;
    }
    /**
     * Generalizes the heuristic function with a common interface
     */
    auto heuristicFunc = [&abstraction](ulonglong color) {
        return abstraction->heuristic((int)color);
    };
    auto t1 = high_resolution_clock::now();
    AStarOpenList openList;
    auto root = abstraction->createNode(startColor);
    auto h = abstraction->heuristic(startColor);
    root.calculateF(0, h);
    openList.insert(root);
    childParent->insert({root.rank, root.rank});
    bool isPathFound = false;
    /**
     * LastMile if pathfinding for the highest level of abstraction or the actual goal of current abstraction lies in the
     * parentGoal region
     */
    bool isLastMile = (!parentGoalColor) || abstraction->unrank(abstraction->getGoalColor()).abstractionColor == parentGoalColor;
    while(!openList.isEmpty()) {
        Node nextNode = openList.removeMinimum();
        closedList->insert(nextNode.rank);

        const AbstractNode &currentNode = abstraction->unrank(nextNode.rank);
        if (abstraction->isGoalReached(currentNode.color) ||
                (!isLastMile && currentNode.abstractionColor == parentGoalColor)) {
            /**
             * For last mile, terminate only if goal is reached.
             * Otherwise, terminate if abstract parent node is reached
             */
            //finalizeNodeLinks(childParent, closedList);
            auto solutionLength = reverseNodeLinks(nextNode.rank, childParent);
            abstraction->setSolutionLength(solutionLength);
            //cout<<"Path Length Abstract World "<<solutionLength<<endl;
            isPathFound = true;
            break;
        }

        // insert child nodes into open list
        for(ulonglong childColor : currentNode.reachableNodes) {
            const auto &childNode = abstraction->unrank(childColor);
            if (abstractParentNodes->empty() || abstractParentNodes->contains(childNode.abstractionColor)) {
                /// Constrained Search
                insertIntoOpenList(openList, closedList, childParent, childColor, nextNode,
                                   heuristicFunc, abstraction->getGCost(childNode, currentNode));
            }
        }
    }
    if(!isPathFound) {
        //cout<<"Abstract Path Not Found"<<endl;
    }

    auto t2 = high_resolution_clock::now();
    auto time_in_ms = duration_cast<chrono::milliseconds>(t2 - t1);
    //cout<<"Total execution time "<<time_in_ms.count()<<endl;
    return isPathFound;
}

//// PRA*
/**
 * If K = K_MAX (10000), PRA*(INF)
 * K < K_MAX , PRA*(K)
 */
bool AStarSearch::partialRefinementAStarSearch(int K, DataPoint &dataPoint) {
    /// container to store paths
    unique_ptr<unordered_map<ulonglong, ulonglong>> pathRealWorld = make_unique<unordered_map<ulonglong, ulonglong>>();

    /// store parent nodes for constrained A*
    unique_ptr<unordered_set<ulonglong>> abstractParentNodes = make_unique<unordered_set<ulonglong>>();

    InitialState initialState{};
    storeInitialState(initialState);
    DataPoint dummy;
    // Used to save the final destination
    int destinationX, destinationY;
    realWorld->getGoal(destinationX, destinationY);
    bool isPathFound = false;
    bool isLastMile = false;
    ulonglong goalColor = 0;
    realWorld->setPathLength(0);

    auto t1 = high_resolution_clock::now();
    while (!isLastMile) {
        /**
         * Search, truncate and refine path through abstraction levels
         */
        if (!searchAndTruncatePathInAbstractWorld(K, abstractParentNodes, goalColor)) {
            break;
        }
        /// Update RealWorld start and goal state. Updating real world start state affects start state of
        /// abstract graphs
        if (realWorld->getMapColors()[destinationX][destinationY] != goalColor) {
            /**
             * Not the goal node
             */
            auto absNextNode = abstractGraph->unrank(goalColor);
            /**
             * Target is centroid of Kth node
             */
            realWorld->setGoalState(absNextNode.centroidRealNode.first, absNextNode.centroidRealNode.second);
        } else {
            /**
             * Target abstract node is the goal node. Restore actual destination coordinates in real world
             * as goal of constrained A* search. We do not need to target the centroid.
             */
            realWorld->setGoalState(destinationX, destinationY);
            /**
             * We will terminate after this, as it is the last real world search
             */
            isLastMile = true;
        }

        /**
         * Search a path from start to goal in the real world using constrained A* search.
         * The goal is set in the previous step. It can either be the actual destination coordinate
         * or an intermediate abstract center.
         */
        if (!searchPathInRealWorldWithAstar(pathRealWorld, abstractParentNodes, dummy, goalColor)) {
            break;
        } else if (isLastMile) {
            isPathFound = true;
        }
        abstractParentNodes->clear();
        goalColor = 0;
    }
    auto t2 = high_resolution_clock::now();
    auto time_in_ms = duration_cast<chrono::milliseconds>(t2 - t1);

    if (isPathFound) {
        //cout<<"PRA* found a path. Total length of path in real world "<<realWorld->getPathLength()<<endl;
        //cout<<"PRA* execution time "<<time_in_ms.count()<<endl;
        //printPathNodes(pathRealWorld, realWorld->getRank(initialState.startX, initialState.startY),
        //               realWorld->getRank(initialState.goalX, initialState.goalY));
        //printRealPath(pathRealWorld, realWorld->getRank(initialState.startX, initialState.startY),
        //                       realWorld->getRank(initialState.goalX, initialState.goalY));
        dataPoint.setPraStarKData(K, realWorld->getPathLength(), (int) time_in_ms.count());
    } else {
        cout<<"PRA* could not find a path to the destination"<<endl;
    }
    restoreInitialState(initialState);
    return isPathFound;
}

unique_ptr<RealWorld> &AStarSearch::accessRealWorld() {
    return realWorld;
}

void AStarSearch::storeInitialState(AStarSearch::InitialState &initState) {
    int startX, startY, goalX, goalY, goalColor;
    realWorld->getStart(startX, startY);
    realWorld->getGoal(goalX, goalY);
    initState.init(startX, startY, goalX, goalY);
}

void AStarSearch::restoreInitialState(AStarSearch::InitialState &initState) {
    realWorld->setStart(initState.startX, initState.startY);
    realWorld->setGoalState(initState.goalX, initState.goalY);
}

void AStarSearch::printPathNodes(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank,
                            ulonglong destinationRank) {

    auto current = childParent->find(rootRank)->first;
    int x, y;
    realWorld->unrank(current, x, y);
    cout<<"("<<x<<","<<y<<") ";
    int count = 1;
    while(current != destinationRank) {
        current = childParent->find(current)->second;
        realWorld->unrank(current, x, y);
        cout<<"("<<x<<","<<y<<") ";
        ++count;
    }
    cout<<endl;
    cout<<"Total Nodes in Path "<<count<<endl;
}

/**
 * Search path through abstraction levels starting from the most abstracted graph.
 * The path is then truncated to K (in case of PRA*(K)) and refined in the lower level
 * of abstraction.
 */
bool
AStarSearch::searchAndTruncatePathInAbstractWorld(int K, unique_ptr<unordered_set<ulonglong>> &abstractParentNodes, ulonglong &parentGoalColor) {
    int abstractionLevel = 6;
    unique_ptr<unordered_map<ulonglong, ulonglong>> path = make_unique<unordered_map<ulonglong, ulonglong>>();
    bool isPathFound = true;
    ulonglong startColor = 0;
    while (abstractionLevel >= 1) {
        /// Search and Refine
        if (!searchPathInAbstractGraphWithAstar(path, abstractParentNodes, abstractionLevel, parentGoalColor, startColor)) {
            isPathFound = false;
            break;
        }
        /// Truncate
        abstractParentNodes->clear();
        ulonglong currentColor = startColor;
        abstractParentNodes->insert(currentColor);
        /**
         * Search the last abstract node on the truncated path. Either it is the Kth node or the goal
         * node.
         */
        // PRA*(Inf) uses K = 10000
        ulonglong goalColorAbstraction = getGoalColorOfAbstraction(abstractionLevel);
        int i = 1;
        for(; i<=K && currentColor != goalColorAbstraction; ++i) {
            currentColor = path->find(currentColor)->second;
            abstractParentNodes->insert(currentColor);
        }
        if (currentColor == goalColorAbstraction && !abstractParentNodes->contains(goalColorAbstraction)) {
            abstractParentNodes->insert(goalColorAbstraction);
        }
        parentGoalColor = currentColor;
        path->clear();
        --abstractionLevel;
    }

    return isPathFound;
}


ulonglong AStarSearch::getGoalColorOfAbstraction(int abstractionLevel) {
    switch (abstractionLevel) {
        case 1:
            return abstractGraph->getGoalColor();
        case 2:
            return abstractGraph2->getGoalColor();
        case 3:
            return abstractGraph3->getGoalColor();
        case 4:
            return abstractGraph4->getGoalColor();
        case 5:
            return abstractGraph5->getGoalColor();
        case 6:
            return abstractGraph6->getGoalColor();
        case 7:
            return abstractGraph7->getGoalColor();
    }
    throw std::runtime_error("Invalid Abstraction Level");
}

/**
 * Check pathability at abstraction level 7
 */
bool AStarSearch::checkPathExists(int &approxLength) {
    unique_ptr<unordered_set<ulonglong>> dummy = make_unique<unordered_set<ulonglong>>();
    unique_ptr<unordered_map<ulonglong, ulonglong>> pathAbstractWorld = make_unique<unordered_map<ulonglong, ulonglong>>();
    ulonglong startColor = 0;
    bool isFound = searchPathInAbstractGraphWithAstar(pathAbstractWorld, dummy, 7, 0, startColor);
    approxLength = (int)round(abstractGraph7->getSolutionLength() * 19.6);
    return isFound;
}

unique_ptr<AbstractGraph_2> &AStarSearch::accessAbstractGraph2() {
    return abstractGraph2;
}

unique_ptr<AbstractGraph_3> &AStarSearch::accessAbstractGraph3() {
    return abstractGraph3;
}

unique_ptr<AbstractGraph_4> &AStarSearch::accessAbstractGraph4() {
    return abstractGraph4;
}

unique_ptr<AbstractGraph_5> &AStarSearch::accessAbstractGraph5() {
    return abstractGraph5;
}

void AStarSearch::copyAbstractNodesFromLevel(int abstractLevel, vector<AbstractNode> &abstractNodes) {
    switch (abstractLevel) {
        case 1:
            abstractNodes = abstractGraph->getAllAbstractNodes();
            break;
        case 2:
            abstractNodes = abstractGraph2->getAllAbstractNodes();
            break;
        case 3:
            abstractNodes = abstractGraph3->getAllAbstractNodes();
            break;
        case 4:
            abstractNodes = abstractGraph4->getAllAbstractNodes();
            break;
        case 5:
            abstractNodes = abstractGraph5->getAllAbstractNodes();
            break;
        case 6:
            abstractNodes = abstractGraph6->getAllAbstractNodes();
            break;
        case 7:
            abstractNodes = abstractGraph7->getAllAbstractNodes();
            break;
    }
}
