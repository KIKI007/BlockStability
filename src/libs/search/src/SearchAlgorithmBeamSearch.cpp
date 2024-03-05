//
// Created by 汪子琦 on 23.03.22.
//

#include "search/SearchAlgorithmBeamSearch.h"
#include <tbb/parallel_for.h>
#include "tbb/tick_count.h"

#include <algorithm>    // std::shuffle
#include <random>       // std::default_random_engine

namespace search
{
double SearchAlgorithmBeamSearch::search(PtrS inputNode, AssemblySequence& sequence)
{
    int step = 0;
    PtrS final_node;
    candidates_.push_back(inputNode);

    while(!candidates_.empty())
    {
        step++;
        std::cout << step << std::endl;

        trimSolution();

        candidates_ = graph_->expandNodes(candidates_);

        for(int id = 0; id < candidates_.size(); id++)
        {
            PtrS prev_node = candidates_[id]->prev_node;
            PtrS curr_node = candidates_[id];

            if(graph_->checkEndNode(curr_node))
            {
                final_node = curr_node;
            }
            else {
                if(final_node)
                {
                    auto prev_size = graph_->state_graph_->getInstalledParts(final_node->assembly_state).size();
                    auto curr_size = graph_->state_graph_->getInstalledParts(curr_node->assembly_state).size();
                    if(prev_size < curr_size) {
                        final_node = curr_node;
                    }
                }
                else {
                    final_node = curr_node;
                }
            }
        }
    }

    if(final_node)
    {
        sequence = getSolution(final_node);
        std::cout << "Found ";
        return 0;
    }

    std::cout << "end" << std::endl;
    return std::numeric_limits<double>::max();
}

double SearchAlgorithmBeamSearch::search(AssemblySequence& sequence) {
    clear();
    auto root = graph_->createRootNode();
    return search(root, sequence);
}

void SearchAlgorithmBeamSearch::trimSolution()
{
    auto engine = std::default_random_engine(10);

    std::shuffle(candidates_.begin(), candidates_.end(), engine);

    std::vector<PtrS> tmp;
    for(int id = 0; id < maxLayerNodeExplore && id < candidates_.size(); id++){
        tmp.push_back(candidates_[id]);
    }
    candidates_ = tmp;

}

}

