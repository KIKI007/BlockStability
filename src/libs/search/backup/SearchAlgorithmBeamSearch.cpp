//
// Created by 汪子琦 on 23.03.22.
//

#include "SearchAlgorithmBeamSearch.h"

#include <tbb/parallel_for.h>

#include "search/StateGraphHolding.h"
#include "tbb/tick_count.h"

#include <algorithm>    // std::shuffle
#include <random>       // std::default_random_engine

namespace search
{
double SearchAlgorithmBeamSearch::search(StateGraph::PtrN inputNode, AssemblySequence& sequence)
{
    int step = 0;
    StateGraph::PtrN final_node;
    double final_cost = std::numeric_limits<double>::max();
    candidates_.push_back(inputNode);

    while(!candidates_.empty())
    {
        step++;
        std::cout << step << std::endl;

        trimSolution();
        std::vector<StateGraphEdge> edges;
        stateGraph_->expandNodes(candidates_, edges);

        candidates_.clear();
        searchNodeInQueue_.clear();

        for(int id = 0; id < edges.size(); id++)
        {
            StateGraph::PtrN prev_node = edges[id].prev_node;
            StateGraph::PtrN next_node = edges[id].next_node;
            double edge_cost = edges[id].edge_cost;
            updateNode(prev_node, next_node, edge_cost);

            if(stateGraph_->checkEndNode(next_node))
            {
                 if(final_cost > next_node->currentCost)
                 {
                     final_cost = next_node->currentCost;
                     final_node = next_node;
                 }
            }
            else {
                if(final_node) {
                    auto prev_size = stateGraph_->getInstalledParts(final_node).size();
                    auto curr_size = stateGraph_->getInstalledParts(next_node).size();
                    if(prev_size < curr_size) {
                        final_node = next_node;
                    }
                }
                else {
                    final_node = next_node;
                }
            }
        }
    }
    if(final_node)
    {
        getSolution(final_node, sequence);
        std::cout << "Found " << ", Cost " << final_node->currentCost << std::endl;
        return final_cost;
    }

    std::cout << "end" << std::endl;
    return std::numeric_limits<double>::max();
}

double SearchAlgorithmBeamSearch::search(AssemblySequence& sequence) {
    clear();
    stateGraph_->clear();
    stateGraph_->createRootNode();
    return search(stateGraph_->nodes_[0], sequence);
}

void SearchAlgorithmBeamSearch::updateNode(StateGraph::PtrN currNode, StateGraph::PtrN updateNode, double edgeCost)
{

    double newCost = currNode->currentCost + edgeCost;

    if(updateNode->currentCost > newCost)
    {
        updateNode->currentCost = newCost;
        updateNode->parent = currNode->nodeID;
    }

    if(searchNodeInQueue_.find(updateNode->nodeID) == searchNodeInQueue_.end())
    {
        searchNodeInQueue_[updateNode->nodeID] = candidates_.size();
        candidates_.push_back(updateNode);
    }
}

void SearchAlgorithmBeamSearch::print(StateGraph::PtrN node)
{
    std::vector<int> partIDs = stateGraph_->getInstalledParts(node);
    auto graph = dynamic_cast<StateGraphHolding*>(stateGraph_.get());
    std::vector<int> fixedIDs = graph->getFixedParts(node);
    for(int id =0 ; id < partIDs.size(); id++){
        std::cout << partIDs[id];
        if(std::find(fixedIDs.begin(), fixedIDs.end(), partIDs[id]) != fixedIDs.end()){
            std::cout << "*";
        }
        std::cout << " ";
    }
    std::cout << std::endl;
}

void SearchAlgorithmBeamSearch::trimSolution()
{
    auto engine = std::default_random_engine(10);

    //sort the canidate note
    std::sort(candidates_.begin(), candidates_.end(), [&](StateGraph::PtrN A, StateGraph::PtrN B)-> bool{
        if(A->currentCost < B->currentCost){
            return true;
        }
        if(std::abs(A->currentCost - B->currentCost) < 1E-6){
            return A->nodeID < B->nodeID;
        }
        return false;
    });

    if(candidates_.empty())
        return;

    int cost = candidates_[std::min(maxLayerNodeExplore, (int)candidates_.size() - 1)]->currentCost;

    for(auto it = candidates_.begin(); it != candidates_.end();)
    {
        if((*it)->currentCost > cost){
            it = candidates_.erase(it);
        }
        else{
            it++;
        }
    }

    std::shuffle(candidates_.begin(), candidates_.end(), engine);

    std::vector<StateGraph::PtrN> tmp;
    for(int id = 0; id < maxLayerNodeExplore && id < candidates_.size(); id++){
        tmp.push_back(candidates_[id]);
    }
    candidates_ = tmp;

}

}

