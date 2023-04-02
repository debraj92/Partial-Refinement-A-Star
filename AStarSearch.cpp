//
// Created by Debraj Ray on 2023-03-27.
//

#include "AStarSearch.h"
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

void AStarSearch::createAbstractGraph(const string &fileName, int startX, int startY, int goalX, int goalY) {
    createRealWorld(fileName, startX, startY, goalX, goalY);
    abstractGraph = make_unique<AbstractGraph>();
    abstractGraph->createAbstractGraph(*realWorld);
    abstractGraph->setGoalColor(realWorld->getMapColors()[goalX][goalY]);
}

void AStarSearch::createRealWorld(const string &fileName, int startX, int startY, int goalX, int goalY) {
    realWorld = make_unique<RealWorld>();
    realWorld->readMapFromFile(fileName);
    realWorld->setGoalState(goalX, goalY);
    realWorld->setStart(startX, startY);
}

/**
 * If abstractParentNodes is empty then this function is plain A* search.
 * Otherwise this is a constrained A* search as required in PRA*.
 */
bool AStarSearch::searchPathInRealWorldWithAstar(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent,
                                                 unique_ptr<unordered_set<ulonglong>> &abstractParentNodes) {
    cout<<"Searching in Real World"<<endl;

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
    int finalGoalColor = abstractGraph->getGoalColor();
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

        if (realWorld->isGoalReached(x, y) ||
                ( /// PRA* Termination
                        !abstractParentNodes->empty() &&
                        /// This Goal is intermediate
                        realWorld->getMapColors()[goalX][goalY] != finalGoalColor &&
                        /// We have reached a (x,y) in the abstract goal node
                        realWorld->getMapColors()[x][y] == realWorld->getMapColors()[goalX][goalY]
                )
        ) {
            auto solutionLength = reverseNodeLinks(nextNode.rank, childParent);
            cout<<"Path Length Real World "<<solutionLength<<endl;
            //printRealPath(childParent, root.rank, nextNode.rank);
            isPathFound = true;
            /**
             * Only required for PRA*
             */
            realWorld->setStart(x, y);
            /**
             * We aggregate path length for PRA*.
             * For A* this has no impact
             */
            realWorld->setPathLength(solutionLength + realWorld->getPathLength());
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
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc);
        }
        // down
        if (x < realWorld->MAX_SIZE - 1 && realWorld->getRealMap()[x+1][y] == 0 &&
                (abstractParentNodes->empty() || abstractParentNodes->contains(realWorld->getMapColors()[x+1][y]))
        ) {
            ulonglong childRank = realWorld->getRank(x+1, y);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc);
        }
        // left
        if (y > 0 && realWorld->getRealMap()[x][y-1] == 0 &&
                (abstractParentNodes->empty() || abstractParentNodes->contains(realWorld->getMapColors()[x][y-1]))
        ) {
            ulonglong childRank = realWorld->getRank(x, y-1);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc);
        }
        // right
        if (y < realWorld->MAX_SIZE - 1 && realWorld->getRealMap()[x][y+1] == 0 &&
                (abstractParentNodes->empty() || abstractParentNodes->contains(realWorld->getMapColors()[x][y+1]))
        ) {
            ulonglong childRank = realWorld->getRank(x, y+1);
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc);
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
                                Node &parentNode,
                                const std::function<int(ulonglong)>& heuristic) {

    // do not insert in open list if node already exists in closed
    if (closedList->contains(childRank)) {
        return;
    }
    Node childNode(childRank);
    if (!openList.isPresent(childNode)) {
        /**
         * New node, never visited. Just insert in open.
         */
        childNode.calculateF(parentNode.g + 1, heuristic(childRank));
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

void AStarSearch::copyRealWorld(vector<vector<int>> &copied) {
    for(int row = 0; row < realWorld->MAX_SIZE; ++row) {
        copied[row].reserve(realWorld->MAX_SIZE);
        for(int col = 0; col < realWorld->MAX_SIZE; ++col) {
            copied[row][col] = realWorld->getRealMap()[row][col];
        }
    }
}


///////// AStar search in Abstract World //////////////////
bool AStarSearch::searchPathInAbstractGraphWithAstar(unique_ptr<unordered_map<ulonglong, ulonglong>> &childParent) {
    cout<<"Searching in Abstract World"<<endl;
    unique_ptr<unordered_set<ulonglong>> closedList = make_unique<unordered_set<ulonglong>>();
    int startX, startY;
    realWorld->getStart(startX, startY);
    int startColor = realWorld->getMapColors()[startX][startY];
    if(startColor == -1) {
        cout<<"Abstract Path Not Found"<<endl;
        return false;
    }
    /**
     * Generalizes the heuristic function with a common interface
     */
    auto heuristicFunc = [this](ulonglong color) {
        return abstractGraph->heuristic((int)color);
    };
    auto t1 = high_resolution_clock::now();
    AStarOpenList openList;
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
            finalizeNodeLinks(childParent, closedList);
            auto solutionLength = reverseNodeLinks(nextNode.rank, childParent);
            cout<<"Path Length Abstract World "<<solutionLength<<endl;
            isPathFound = true;
            //printAbstractPath(childParent, root.rank);
            break;
        }

        // insert child nodes into open list
        for(int childColor : currentNode.reachableNodes) {
            ulonglong childRank = childColor;
            insertIntoOpenList(openList, closedList, childParent, childRank, nextNode, heuristicFunc);
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
    cout<<"Abstract Path Colours "<<endl;
    cout<<current<<" ";
    while(!abstractGraph->isGoalReached((int)current)) {
        current = childParent->find(current)->second;
        cout<<current<<" ";
    }
    cout<<endl;

    /*
     * //// Prints Abstract path in real world
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
     */
}

//// PRA*
/**
 * If K = 0, PRA*(INF)
 * K > 0 , PRA*(K)
 */
bool AStarSearch::partialRefinementAStarSearch(int K) {

    unique_ptr<unordered_map<ulonglong, ulonglong>> pathRealWorld = make_unique<unordered_map<ulonglong, ulonglong>>();
    unique_ptr<unordered_map<ulonglong, ulonglong>> pathAbstractWorld = make_unique<unordered_map<ulonglong, ulonglong>>();

    unique_ptr<unordered_set<ulonglong>> abstractParentNodes = make_unique<unordered_set<ulonglong>>();

    int currentX, currentY;
    realWorld->getStart(currentX, currentY);
    assert(realWorld->getRealMap()[currentX][currentY] != 1 && realWorld->getMapColors()[currentX][currentY] != -1);

    // Used to save the final destination
    int destinationX, destinationY;
    realWorld->getGoal(destinationX, destinationY);
    bool isPathFound = false;
    bool isLastMile = false;
    while (!isLastMile) {
        pathAbstractWorld->clear();
        if (!searchPathInAbstractGraphWithAstar(pathAbstractWorld)) {
            break;
        }

        if (K > 0) {
            /**
            * The abstract path is truncated to K nodes by loading relevant nodes in
             * abstractParentNodes. The abstractParentNodes is then used in constrained A* search
             * in real world as needed in PRA*.
            */
            abstractParentNodes->clear();
            ulonglong currentColor = realWorld->getMapColors()[currentX][currentY];
            abstractParentNodes->insert(currentColor);
            /**
             * Search the last abstract node on the truncated path. Either it is the Kth node or the goal
             * node.
             */
            for(int i=1; i<K && currentColor != abstractGraph->getGoalColor(); ++i) {
                currentColor = pathAbstractWorld->find(currentColor)->second;
                abstractParentNodes->insert(currentColor);
            }

            if ((ulonglong)abstractGraph->getGoalColor() != currentColor) {
                /**
                 * Not the goal node
                 */
                auto absNextNode = abstractGraph->unrank(currentColor);
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
        } else {
            /**
             * PRA* inf. There is only one real world search
             */
            ulonglong currentColor = realWorld->getMapColors()[currentX][currentY];
            abstractParentNodes->insert(currentColor);
            /**
             * Just to populate abstractParentNodes with the abstract nodes found in abstract A* search.
             * This maintains uniformity in the constrained A* search between PRA*(K) and PRA*(inf)
             */
            while(currentColor != abstractGraph->getGoalColor()) {
                currentColor = pathAbstractWorld->find(currentColor)->second;
                abstractParentNodes->insert(currentColor);
            }
            isLastMile = true;
        }

        if (!searchPathInRealWorldWithAstar(pathRealWorld, abstractParentNodes)) {
            break;
        } else if (isLastMile) {
            isPathFound = true;
        }
        realWorld->getStart(currentX, currentY);
    }
    if (isPathFound) {
        cout<<"PRA* found a path. Total length of path in real world "<<realWorld->getPathLength()<<endl;
    } else {
        cout<<"PRA* could not find a path to the destination"<<endl;
    }

    return isPathFound;
}
