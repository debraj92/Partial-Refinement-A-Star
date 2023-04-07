//
// Created by Debraj Ray on 2023-04-04.
//

#include "AbstractGraph_2.h"

void AbstractGraph_2::createAbstractGraphNodes() {
    auto &abGraphMap = abGraph.accessAbstractGraph();
    int color = 0;
    //cout<<"Total colors in ABG1 "<<abGraphMap.size()<<endl;
    for(int abGColor = 1; abGColor <= abGraphMap.size(); ++abGColor) {
        auto &abNode = abGraphMap.find(abGColor)->second;
        if (abNode.abstractionColor >= 0) {
            continue;
        }
        ++color;

        /**
         * Find two nodes that are connected to each other through a 3rd node. Then all the 4 nodes form a clique in the
         * next abstraction level. They will be given a single color in the next level.
         */
        bool found4Clique = false;
        bool found3Clique = false;
        bool found2Clique = false;
        ulonglong two_clique[2];
        ulonglong three_clique[3];
        /**
         * Find a connected node and see if its already colored. If No, then this is at least a two clique with abNode
         */
        for(auto connected1_itr = abNode.reachableNodes.begin(); connected1_itr != abNode.reachableNodes.end(); ++connected1_itr) {
            auto &abNode1 = abGraph.unrank(*connected1_itr);
            if (abNode1.abstractionColor >= 0) {
                // already colored
                /**
                 * This abstract graph node already belongs to a node of abstract graph 2
                 */
                continue;
            } else {
                found2Clique = true;
                two_clique[0] = abNode.color;
                two_clique[1] = abNode1.color;
            }
            /**
             * Find another connected node abNode2 and see if its already colored. If No, then this is at least a three clique
             * with abNode and abNode1. We need to ensure we are not picking abNode1 again.
            */
            for(auto connected2_itr = abNode.reachableNodes.begin(); connected2_itr != abNode.reachableNodes.end(); ++connected2_itr) {
                if (*connected2_itr == *connected1_itr) {
                    // Both are the same node.
                    continue;
                }
                auto &abNode2 = abGraph.unrank(*connected2_itr);
                if (abNode2.abstractionColor >= 0) {
                    // already colored
                    continue;
                } else {
                    // we have two different uncolored reachable nodes of abNode. These are also at distance 1 from abNode.
                    // at least a 3 clique
                    found3Clique = true;
                    three_clique[0] = abNode.color;
                    three_clique[1] = abNode1.color;
                    three_clique[2] = abNode2.color;
                }
                /**
                 * Find the last node which is in the intersection of abNode1 and abNode2. It should also not be abNode.
                 * For this, we check all reachable nodes from abNode2. Here we will reach abNode and may be abNode1.
                 * These two cases are not interesting. Any other uncolored node abNode3 is a potential 4 clique candidate.
                 * The final check for intersection is to see if abNode3 is reachable from abNode1. If yes, 4 clique.
                */
                for(auto connected3_itr = abNode2.reachableNodes.begin(); connected3_itr != abNode2.reachableNodes.end(); ++connected3_itr) {
                    if (*connected3_itr == abNode.color || *connected3_itr == abNode1.color) {
                        continue;
                    }
                    auto &abNode3 = abGraph.unrank(*connected3_itr);
                    if (abNode3.abstractionColor >= 0) {
                        // already colored
                        continue;
                    }
                    /**
                     * Is reachable from abNode1?
                     */
                    if (abNode1.reachableNodes.contains(abNode3.color)) {
                        /// abNode3 in intersection of abNode1 and abNode2
                        /**
                         * We have found the 4 nodes clique
                         */
                        abNode.abstractionColor = color;
                        abNode1.abstractionColor = color;
                        abNode2.abstractionColor = color;
                        abNode3.abstractionColor = color;
                        colorAbstractNodeMap.insert({color, {color, abNode.centroidRealNode}});
                        found4Clique = true;
                        break;
                    }
                }
                if (found4Clique) break;
            }
            if (found4Clique) break;
        }
        /**
         * Form 3 clique or 2 clique or an orphan node if 4 clique not found
         */
        if (!found4Clique) {
            if (found3Clique) {
                abGraph.unrank(three_clique[0]).abstractionColor = color;
                abGraph.unrank(three_clique[1]).abstractionColor = color;
                abGraph.unrank(three_clique[2]).abstractionColor = color;
                colorAbstractNodeMap.insert({color, {color, abNode.centroidRealNode}});
            } else if (found2Clique) {
                abGraph.unrank(two_clique[0]).abstractionColor = color;
                abGraph.unrank(two_clique[1]).abstractionColor = color;
                colorAbstractNodeMap.insert({color, {color, abNode.centroidRealNode}});
            } else {
                abNode.abstractionColor = color;
                colorAbstractNodeMap.insert({color, {color, abNode.centroidRealNode}});
            }
        }
    }
    //cout<<"Total Colors in ABG2 "<<color<<endl;
}

/**
 * Given a AbstractGraph2 node N_2 (identified by its color).
 * DFS through AbstractGraph nodes N. N belongs to N_2 if N.abstractionColor == N_2.color
 * If N belongs to N_2 then DFS for all N' connected to N.
 * If N does not belong to N_2, then identify parent of N using its abstractionColor. Let that be N_2'
 * => We will connect N_2 and N_2'
 *
 * Visited should store color of AbstractGraph nodes.
 * abG2Color is the color of the AbstractGraph2 node which is the parent of the AbstractGraph nodes
 * AbstractNode refers to AbstractGraph node.
 */
void AbstractGraph_2::dfsToConnectAbstractNodes(const AbstractNode &abNode, int abG2Color, unordered_set<int> visited) {
    if (!colorAbstractNodeMap.contains(abG2Color)) {
        return;
    }

    if (visited.contains(abNode.color)) {
        return;
    }
    visited.insert(abNode.color);

    for(const auto& connectedChildNodeColor : abNode.reachableNodes) {
        auto &connectedChildNode = abGraph.unrank(connectedChildNodeColor);
        assert(connectedChildNode.abstractionColor >= 0);
        if (connectedChildNode.abstractionColor == abG2Color) {
            dfsToConnectAbstractNodes(connectedChildNode, abG2Color, visited);
        } else {
            createUndirectedEdge(abG2Color, connectedChildNode.abstractionColor);
        }
    }
}

/**
 * We will DFS through nodes in abstractGraph. Based on connectivity at abstractGraph level we connect nodes at
 * abstractGraph2 level
 */
void AbstractGraph_2::createUndirectedEdges() {
    int abG2Color = 1;
    unordered_set<int> visited;
    /**
     * Check if each node in abstractGraph2 is processed
     */
    while (colorAbstractNodeMap.contains(abG2Color)) {
        /**
         * We need to extract a node from abstractGraph which belongs to this node of abstractGraph2.
         * The centroid of abstractGraph2 is same as abstractGraph and points to location in realWorld.
         * We use this centroid to find the color of this node painted by the parent abstractGraph node. This
         * abstractGraph node must belong to the current abstractGraph2 node and hence is the starting point of DFS.
         */
        const auto &centroid = colorAbstractNodeMap[abG2Color].centroidRealNode;
        /// centroid is common for both abstract graph and abstract graph2
        /// We can use it to find the corresponding abstract graph node
        /// if we want to further move up abstraction levels using centroid, we just have to get the parent
        /// node color using abGNode.abstractionColor
        auto abGColor = rworld.getMapColors()[centroid.first][centroid.second];
        if (!visited.contains(abGColor)) {
            const auto &abGNode = abGraph.unrank(abGColor);
            dfsToConnectAbstractNodes(abGNode, abG2Color, visited);
        }
        //printNode(abG2Color);
        ++abG2Color;
    }
}

/**
 * Create an edge between nodes in abstractGraph2
 */
void AbstractGraph_2::createUndirectedEdge(int color1, int color2) {
    colorAbstractNodeMap.find(color1)->second.addChildAbstractNode(color2);
    colorAbstractNodeMap.find(color2)->second.addChildAbstractNode(color1);
}

void AbstractGraph_2::setGoalColor(int color) {
    goalColor = color;
}

double AbstractGraph_2::heuristic(int nodeColor) {
    const auto& centroidCurrent = colorAbstractNodeMap.find(nodeColor)->second.centroidRealNode;
    const auto &centroidGoal = colorAbstractNodeMap.find(goalColor)->second.centroidRealNode;
    /**
     * Formula:
     * ||delta x| - |delta y|| + 1.4142 * min(|delta x|, |delta y|)
     */
    double delta_x = abs(centroidCurrent.first - centroidGoal.first);
    double delta_y = abs(centroidCurrent.second - centroidGoal.second);
    return abs(delta_x - delta_y) + sqrt(2) * min(delta_x, delta_y);
}

Node AbstractGraph_2::createNode(int color) {
    return {(ulonglong) color};
}

bool AbstractGraph_2::isGoalReached(int color) {
    return goalColor == color;
}

AbstractNode &AbstractGraph_2::unrank(ulonglong rank) {
    assert(colorAbstractNodeMap.find((int)rank) != colorAbstractNodeMap.end());
    return colorAbstractNodeMap.find((int)rank)->second;
}

int AbstractGraph_2::getGoalColor() {
    return goalColor;
}

vector<AbstractNode> AbstractGraph_2::getAllAbstractNodes() {
    vector<AbstractNode> allNodes;
    for (auto& [color, abNode]: colorAbstractNodeMap) {
        allNodes.emplace_back(abNode);
    }
    return move(allNodes);
}

void AbstractGraph_2::createAbstractGraph() {
    createAbstractGraphNodes();
    createUndirectedEdges();
}

void AbstractGraph_2::printNode(int color) {
    const auto &color_AbsNode = colorAbstractNodeMap.find(color);
    cout<<"Color: "<<color_AbsNode->first<<", Connected Colors: ";
    for(const auto& childColor: color_AbsNode->second.reachableNodes) {
        cout<<childColor<<"  ";
    }
    cout<<endl;
}

int AbstractGraph_2::getStartColor() {
    int startX, startY;
    rworld.getStart(startX, startY);
    int startColor_abstraction1 = rworld.getMapColors()[startX][startY];
    return abGraph.unrank(startColor_abstraction1).abstractionColor;
}

unordered_map<int, AbstractNode> &AbstractGraph_2::accessAbstractGraph() {
    return colorAbstractNodeMap;
}
