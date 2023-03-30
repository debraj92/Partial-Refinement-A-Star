//
// Created by Debraj Ray on 2023-03-27.
//

#include "AStarSearch.h"
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

void AStarSearch::createAbstractGraph(const string &fileName, int goalX, int goalY) {
    createRealWorld(fileName, goalX, goalY);
    abstractGraph = make_unique<AbstractGraph>();
    abstractGraph->createAbstractGraph(*realWorld);
    abstractGraph->setGoalColor(realWorld->getMapColors()[goalX][goalY]);
}

void AStarSearch::createRealWorld(const string &fileName, int goalX, int goalY) {
    realWorld = make_unique<RealWorld>();
    realWorld->readMapFromFile(fileName);
    realWorld->setGoalState(goalX, goalY);
}

bool AStarSearch::searchPathInRealWorldWithAstar(int startX, int startY) {
    cout<<"Starting Search in Real World"<<endl;
    auto t1 = high_resolution_clock::now();
    AStarOpenList openList;
    unique_ptr<unordered_set<ulonglong>> closedList = make_unique<unordered_set<ulonglong>>();
    unique_ptr<unordered_map<ulonglong, ulonglong>> childParent = make_unique<unordered_map<ulonglong, ulonglong>>();
    auto root = realWorld->createNode(startX, startY);
    auto h = realWorld->heuristic(startX, startY);
    root.calculateF(0, h);
    openList.insert(root);
    childParent->insert({root.rank, root.rank});
    int x, y;
    bool isPathFound = false;
    while(!openList.isEmpty()) {
        Node nextNode = openList.removeMinimum();
        closedList->insert(nextNode.rank);

        // un-rank
        realWorld->unrank(nextNode.rank, x, y);
        if (realWorld->isGoalReached(x, y)) {
            cout<<"Goal Reached "<<endl;
            finalizeNodeLinks(childParent, closedList);
            auto solutionLength = reverseNodeLinks(nextNode.rank, childParent);
            cout<<"Path Length "<<solutionLength<<endl;
            printRealPath(childParent, root.rank);
            isPathFound = true;
            break;
        }

        // generate moves
        // up
        if (x > 0 && realWorld->getRealMap()[x-1][y] == 0) {
            ulonglong childRank = realWorld->getRank(x-1, y);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode);
        }
        // down
        if (x < realWorld->MAX_SIZE - 1 && realWorld->getRealMap()[x+1][y] == 0) {
            ulonglong childRank = realWorld->getRank(x+1, y);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode);
        }
        // left
        if (y > 0 && realWorld->getRealMap()[x][y-1] == 0) {
            ulonglong childRank = realWorld->getRank(x, y-1);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode);
        }
        // right
        if (y < realWorld->MAX_SIZE - 1 && realWorld->getRealMap()[x][y+1] == 0) {
            ulonglong childRank = realWorld->getRank(x, y+1);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode);
        }
    }
    if (!isPathFound) {
        cout<<"Path Not Found"<<endl;
    }

    auto t2 = high_resolution_clock::now();
    auto time_in_ms = duration_cast<chrono::milliseconds>(t2 - t1);
    cout<<"Total execution time "<<time_in_ms.count()<<endl;

    return isPathFound;
}

void
AStarSearch::insertIntoOpenList(AStarOpenList &openList,
                                unique_ptr<unordered_set<ulonglong>> &closedList,
                                unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent,
                                ulonglong childRank,
                                Node &parentNode) {

    // do not insert in open list if node already exists in closed
    if (closedList->contains(childRank)) {
        return;
    }
    Node childNode(childRank);
    if (!openList.isPresent(childNode)) {
        /**
         * New node, never visited. Just insert in open.
         */
        childNode.calculateF(parentNode.g + 1, realWorld->heuristic(childRank));
        openList.insert(childNode);
        // child -> parent
        childParent->insert({childNode.rank, parentNode.rank});
    } else {
        /**
         * Node exists in open. Insert if better path found
         */
        if (openList.updateIfBetterPath(childNode, parentNode.g + 1)) {
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
    unique_ptr<unordered_map<ulonglong, ulonglong>> temp_childParent = make_unique<unordered_map<ulonglong, ulonglong>>();

    auto parent = childParent->find(current)->second;
    temp_childParent->insert(make_pair(parent, current));
    int count = 0;
    while(parent !=  current) {
        current = parent;
        parent = childParent->find(parent)->second;
        temp_childParent->insert(make_pair(parent, current));
        count++;
    }
    childParent.reset();
    childParent = std::move(temp_childParent);
    return count+1;
}

void AStarSearch::printRealPath(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank) {
    vector<vector<int>> copied(realWorld->MAX_SIZE);
    copyRealWorld(copied);
    int x, y;
    auto current = childParent->find(rootRank)->first;
    realWorld->unrank(current, x, y);
    copied[x][y] = 9;
    while(!realWorld->isGoalReached(x, y)) {
        current = childParent->find(current)->second;
        realWorld->unrank(current, x, y);
        copied[x][y] = 9;
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
bool AStarSearch::searchPathInAbstractGraphWithAstar(int startX, int startY) {
    cout<<"Starting Search in Abstract World"<<endl;
    int startColor = realWorld->getMapColors()[startX][startY];
    if(startColor == -1) {
        cout<<"Path Not Found"<<endl;
        return false;
    }
    auto t1 = high_resolution_clock::now();
    AStarOpenList openList;
    unique_ptr<unordered_set<ulonglong>> closedList = make_unique<unordered_set<ulonglong>>();
    unique_ptr<unordered_map<ulonglong, ulonglong>> childParent = make_unique<unordered_map<ulonglong, ulonglong>>();
    auto root = abstractGraph->createNode(startColor);
    auto h = abstractGraph->heuristic(startColor);
    root.calculateF(0, h);
    openList.insert(root);
    childParent->insert({root.rank, root.rank});
    bool isPathFound = false;
    while(!openList.isEmpty()) {
        Node nextNode = openList.removeMinimum();
        closedList->insert(nextNode.rank);

        //unranking
        AbstractNode currentNode = abstractGraph->unrank(nextNode.rank);
        if (abstractGraph->isGoalReached(currentNode.color)) {
            cout<<"Goal Reached "<<endl;
            finalizeNodeLinks(childParent, closedList);
            auto solutionLength = reverseNodeLinks(nextNode.rank, childParent);
            cout<<"Path Length "<<solutionLength<<endl;
            isPathFound = true;
            printAbstractPath(childParent, root.rank);
            break;
        }

        // insert child nodes into open list
        for(int childColor : currentNode.reachableNodes) {
            ulonglong childRank = childColor;
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode);
        }
    }
    if(!isPathFound) {
        cout<<"Path Not Found"<<endl;
    }

    auto t2 = high_resolution_clock::now();
    auto time_in_ms = duration_cast<chrono::milliseconds>(t2 - t1);
    cout<<"Total execution time "<<time_in_ms.count()<<endl;
    return isPathFound;
}

void AStarSearch::printAbstractPath(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent, ulonglong rootRank) {
    auto current = childParent->find(rootRank)->first;
    cout<<"Path Colours "<<endl;
    cout<<current<<" ";
    while(!abstractGraph->isGoalReached((int)current)) {
        current = childParent->find(current)->second;
        cout<<current<<" ";
    }
    cout<<endl;

    vector<vector<int>> copied(realWorld->MAX_SIZE);
    copyRealWorld(copied);
    current = childParent->find(rootRank)->first;
    auto abstractNode = abstractGraph->unrank(current);
    copied[abstractNode.centroidRealNode.first][abstractNode.centroidRealNode.second] = 9;
    while(!abstractGraph->isGoalReached((int)current)) {
        current = childParent->find(current)->second;
        abstractNode = abstractGraph->unrank(current);
        copied[abstractNode.centroidRealNode.first][abstractNode.centroidRealNode.second] = 9;
    }
    realWorld->printProvidedRealMap(copied);
}

//// PRA*
bool AStarSearch::constrainedAStarSearch(int startX, int startY, int K) {
    return false;
}
