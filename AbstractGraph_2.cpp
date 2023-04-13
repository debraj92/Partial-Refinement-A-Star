//
// Created by Debraj Ray on 2023-04-04.
//

#include "AbstractGraph_2.h"
#include <fstream>

void AbstractGraph_2::createAbstractGraphNodes() {
    auto &abGraphMap = abGraph.accessAbstractGraph();
    ulonglong color = 0;
    for(ulonglong abGColor = 1; abGColor <= abGraphMap.size(); ++abGColor) {
        auto &abNode = abGraphMap.find(abGColor)->second;
        if (abNode.abstractionColor > 0) {
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
            if (abNode.totalNodesInRepresentation + abNode1.totalNodesInRepresentation > MAX_NODES) continue;

            double edge1 = abGraph.findShortestDistanceBetweenNodes(abNode1, abNode);
            if (edge1 > MAX_EDGE_LENGTH) continue;

            if (abNode1.abstractionColor > 0) {
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
            bool skip = true;
            for(auto connected2_itr = abNode.reachableNodes.begin(); connected2_itr != abNode.reachableNodes.end(); ++connected2_itr) {
                if(*connected2_itr == *connected1_itr) {
                    skip = false;
                    continue;
                }
                if (skip) continue;

                auto &abNode2 = abGraph.unrank(*connected2_itr);
                if (abNode2.abstractionColor > 0 || !abNode2.reachableNodes.contains(*connected1_itr)) {
                    // already colored
                    continue;
                }
                if (abNode.totalNodesInRepresentation + abNode1.totalNodesInRepresentation
                    + abNode2.totalNodesInRepresentation > MAX_NODES) continue;

                double edge2 = abGraph.findShortestDistanceBetweenNodes(abNode2, abNode);
                if (edge2 > MAX_EDGE_LENGTH) continue;

                if (abGraph.findShortestDistanceBetweenNodes(abNode2, abNode1) < max(edge1, edge2)) {
                    /**
                     * 3 clique must have diagonal edge longer than side edges
                     */
                    continue;
                }
                // we have two different uncolored reachable nodes of abNode. These are also at distance 1 from abNode.
                // at least a 3 clique
                found3Clique = true;
                three_clique[0] = abNode.color;
                three_clique[1] = abNode1.color;
                three_clique[2] = abNode2.color;

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
                    if (abNode3.abstractionColor > 0) {
                        // already colored
                        continue;
                    }
                    /**
                     * Is reachable from abNode1?
                     */
                    if (!abNode1.reachableNodes.contains(abNode3.color) || !abNode.reachableNodes.contains(abNode3.color)) {
                        continue;
                    }
                    if (abNode.totalNodesInRepresentation + abNode1.totalNodesInRepresentation
                        + abNode2.totalNodesInRepresentation + abNode3.totalNodesInRepresentation > MAX_NODES) continue;

                    double edge3 = abGraph.findShortestDistanceBetweenNodes(abNode1, abNode3);
                    double edge4 = abGraph.findShortestDistanceBetweenNodes(abNode2, abNode3);
                    if (edge3 > MAX_EDGE_LENGTH || edge4 > MAX_EDGE_LENGTH) continue;
                    if (abGraph.findShortestDistanceBetweenNodes(abNode, abNode3) < max(edge3, edge4)) {
                        /**
                        * 4 clique must have diagonal edge longer than side edges
                        */
                        continue;
                    }
                    /// abNode3 in intersection of abNode1 and abNode2
                    /**
                     * We have found the 4 nodes clique
                     */
                    abNode.abstractionColor = color;
                    abNode1.abstractionColor = color;
                    abNode2.abstractionColor = color;
                    abNode3.abstractionColor = color;
                    /**
                     * X and Y averages
                     */
                    int totalNodes = abNode.totalNodesInRepresentation + abNode1.totalNodesInRepresentation +
                                     abNode2.totalNodesInRepresentation + abNode3.totalNodesInRepresentation;
                    double xAvg = (double) (abNode.getXTotalInRepresentation() + abNode1.getXTotalInRepresentation() +
                                            abNode2.getXTotalInRepresentation() + abNode3.getXTotalInRepresentation()) / totalNodes;
                    double yAvg = (double) (abNode.getYTotalInRepresentation() + abNode1.getYTotalInRepresentation() +
                                            abNode2.getYTotalInRepresentation() + abNode3.getYTotalInRepresentation()) / totalNodes;
                    pair<double, double> representation = {xAvg, yAvg};
                    colorAbstractNodeMap.insert({color, {color, abNode.centroidRealNode, representation, totalNodes}});
                    found4Clique = true;
                    break;
                }
                if (found4Clique) break;
            }
            if (found4Clique) break;
        }
        /**
         * Form 3 clique or 2 clique or an orphan node if 4 clique not found
         */
        if (!found4Clique) {
            /**
             * For measuring averages across abstract regions. This is then used to calculate the central
             * representation
             */
            double xAvg, yAvg, xSum = 0, ySum = 0;
            int totalNodes = 0;

            if (found3Clique) {
                auto &node1 = abGraph.unrank(three_clique[0]);
                auto &node2 = abGraph.unrank(three_clique[1]);
                auto &node3 = abGraph.unrank(three_clique[2]);

                node1.abstractionColor = color;
                xSum += node1.getXTotalInRepresentation();
                ySum += node1.getYTotalInRepresentation();
                totalNodes += node1.totalNodesInRepresentation;

                node2.abstractionColor = color;
                xSum += node2.getXTotalInRepresentation();
                ySum += node2.getYTotalInRepresentation();
                totalNodes += node2.totalNodesInRepresentation;

                node3.abstractionColor = color;
                xSum += node3.getXTotalInRepresentation();
                ySum += node3.getYTotalInRepresentation();
                totalNodes += node3.totalNodesInRepresentation;

            } else if (found2Clique) {
                auto &node1 = abGraph.unrank(two_clique[0]);
                auto &node2 = abGraph.unrank(two_clique[1]);

                node1.abstractionColor = color;
                xSum += node1.getXTotalInRepresentation();
                ySum += node1.getYTotalInRepresentation();
                totalNodes += node1.totalNodesInRepresentation;

                node2.abstractionColor = color;
                xSum += node2.getXTotalInRepresentation();
                ySum += node2.getYTotalInRepresentation();
                totalNodes += node2.totalNodesInRepresentation;

            } else {
                abNode.abstractionColor = color;

                xSum += abNode.getXTotalInRepresentation();
                ySum += abNode.getYTotalInRepresentation();
                totalNodes += abNode.totalNodesInRepresentation;
            }

            xAvg = xSum / totalNodes;
            yAvg = ySum / totalNodes;
            pair<double, double> representation = {xAvg, yAvg};
            colorAbstractNodeMap.insert({color, {color, abNode.centroidRealNode, representation, totalNodes}});
        }
        assert(abNode.abstractionColor > 0);
    }
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
void AbstractGraph_2::dfsToConnectAbstractNodes(const AbstractNode &abNode, ulonglong abG2Color, unordered_set<ulonglong> &visited) {
    if (!colorAbstractNodeMap.contains(abG2Color)) {
        return;
    }

    if (visited.contains(abNode.color)) {
        return;
    }
    visited.insert(abNode.color);

    for(const auto& connectedChildNodeColor : abNode.reachableNodes) {
        auto &connectedChildNode = abGraph.unrank(connectedChildNodeColor);
        assert(connectedChildNode.abstractionColor > 0);
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
    ulonglong abG2Color = 1;
    unordered_set<ulonglong> visited;
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
void AbstractGraph_2::createUndirectedEdge(ulonglong color1, ulonglong color2) {
    colorAbstractNodeMap.find(color1)->second.addChildAbstractNode(color2);
    colorAbstractNodeMap.find(color2)->second.addChildAbstractNode(color1);
}

void AbstractGraph_2::setGoalColor(ulonglong color) {
    goalColor = color;
}

Node AbstractGraph_2::createNode(ulonglong color) {
    return {(ulonglong) color};
}

bool AbstractGraph_2::isGoalReached(ulonglong color) {
    return goalColor == color;
}

AbstractNode &AbstractGraph_2::unrank(ulonglong rank) {
    assert(colorAbstractNodeMap.find(rank) != colorAbstractNodeMap.end());
    return colorAbstractNodeMap.find(rank)->second;
}

ulonglong AbstractGraph_2::getGoalColor() {
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

void AbstractGraph_2::printNode(ulonglong color) {
    const auto &color_AbsNode = colorAbstractNodeMap.find(color);
    cout<<"Color: "<<color_AbsNode->first<<", Connected Colors: ";
    for(const auto& childColor: color_AbsNode->second.reachableNodes) {
        cout<<childColor<<"  ";
    }
    cout<<endl;
}

ulonglong AbstractGraph_2::getStartColor() {
    int startX, startY;
    rworld.getStart(startX, startY);
    ulonglong startColor_abstraction1 = rworld.getMapColors()[startX][startY];
    return abGraph.unrank(startColor_abstraction1).abstractionColor;
}

unordered_map<ulonglong, AbstractNode> &AbstractGraph_2::accessAbstractGraph() {
    return colorAbstractNodeMap;
}

void AbstractGraph_2::printConnectedColors() {
    ofstream myfile;
    myfile.open ("/Users/debrajray/MyComputer/658/project/colors.csv");
    for(int i=0; i<rworld.MAX_SIZE; ++i) {
        for(int j=0; j<rworld.MAX_SIZE; ++j) {
            ulonglong abG2Color = 0;
            if (rworld.getRealMap()[i][j] == 0) {
                auto abGColor = rworld.getMapColors()[i][j];
                abG2Color = abGraph.unrank(abGColor).abstractionColor;
            }
            if(!abG2Color) {
                myfile<<".   ";
            } else {
                myfile<<abG2Color;
                if (abG2Color < 10) {
                    myfile<<"   ";
                } else if (abG2Color < 100) {
                    myfile<<"  ";
                } else if (abG2Color >= 100) {
                    myfile<<" ";
                }
            }
        }
        myfile<<endl;
    }
    myfile.close();
}