//
// Created by 汪子琦 on 16.03.22.
//

#include "search/SearchAlgorithm.h"

namespace search
{

void SearchAlgorithm::getSolution(StateGraph::PtrN final_node, AssemblySequence &sequence)
{
    double cost = 0.0;
    StateGraph::PtrN node = final_node;
    std::vector<StateGraph::PtrN> node_lists;
    while (true)
    {
        node_lists.push_back(node);
        if(node->parent == -1)
        {
            break;
        }
        else{
            StateGraph::PtrN parentNode = stateGraph_->nodes_[node->parent];
            node = parentNode;
        }
    }

    std::reverse(node_lists.begin(), node_lists.end());
    stateGraph_->getSolution(node_lists, sequence);
    return;
}
}


