//
// Created by 汪子琦 on 16.03.22.
//

#ifndef ROBO_CRAFT_SEARCHALGORITHM_H
#define ROBO_CRAFT_SEARCHALGORITHM_H

#include <map>
#include <queue>
#include <set>

#include "AssemblySequence.h"
#include "StateGraph.h"

namespace search
{
class SearchAlgorithm
{
public:

    std::shared_ptr<StateGraph> stateGraph_;

    std::vector<StateGraph::PtrN> candidates_;

    std::unordered_map<int, int> searchNodeInQueue_;

public:

    SearchAlgorithm(std::shared_ptr<StateGraph> stateGraph) : stateGraph_(stateGraph) {}

public:

    void clear() {
        candidates_.clear();
        searchNodeInQueue_.clear();
        stateGraph_->clear();
    }

    virtual double search(AssemblySequence &sequence){return -1.0;}

    void getSolution(StateGraph::PtrN node, AssemblySequence &sequence);

    //debug
    virtual void print(StateGraph::PtrN node){};
};
}

#endif  //ROBO_CRAFT_SEARCHALGORITHM_H
