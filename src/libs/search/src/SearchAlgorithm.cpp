//
// Created by 汪子琦 on 16.03.22.
//

#include "search/SearchAlgorithm.h"

namespace search
{

AssemblySequence SearchAlgorithm::getSolution(PtrS final_node)
{
    PtrS node = final_node;
    std::vector<PtrS> node_lists;
    while (true)
    {
        node_lists.push_back(node);
        if(node->prev_node == nullptr)
        {
            break;
        }
        else{
            node = node->prev_node;
        }
    }

    std::reverse(node_lists.begin(), node_lists.end());
    return graph_->getSolution(node_lists);

}
}


