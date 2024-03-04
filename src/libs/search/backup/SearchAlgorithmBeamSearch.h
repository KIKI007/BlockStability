//
// Created by 汪子琦 on 23.03.22.
//

#ifndef ROBO_CRAFT_SEARCHALGORITHMBEAMSEARCH_H
#define ROBO_CRAFT_SEARCHALGORITHMBEAMSEARCH_H
#include "../include/search/SearchAlgorithm.h"
namespace search
{
class SearchAlgorithmBeamSearch: public SearchAlgorithm
{
public:

    int maxLayerNodeExplore = 10000;

    SearchAlgorithmBeamSearch(std::shared_ptr<StateGraph> stateGraph,
                              int maxLayerNodeExplore):
          SearchAlgorithm(stateGraph),
          maxLayerNodeExplore(maxLayerNodeExplore)
    {

    }

public:

    double search(AssemblySequence &sequence) override;

    double search(StateGraph::PtrN inputNode, AssemblySequence &sequence);

    void updateNode(StateGraph::PtrN currNode, StateGraph::PtrN updateNode, double edgeCost);

    void print(StateGraph::PtrN node) override;

    void trimSolution();

};
}


#endif  //ROBO_CRAFT_SEARCHALGORITHMBEAMSEARCH_H
