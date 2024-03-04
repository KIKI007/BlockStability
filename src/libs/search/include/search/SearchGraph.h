//
// Created by Ziqi Wang on 04.03.2024.
//

#ifndef SEARCHGRAPH_H
#define SEARCHGRAPH_H

#include "StateGraph.h"

struct SearchNode {
public:
    search::StateGraph::PtrN assembly_state;
    std::vector<int> robot_states;
};

struct SearchEdge {
public:
    std::shared_ptr<SearchNode> prev_node, next_node;
};

class SearchGraph {

};



#endif //SEARCHGRAPH_H
