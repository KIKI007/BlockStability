//
// Created by 汪子琦 on 23.03.22.
//

#ifndef ROBO_CRAFT_SEARCHALGORITHMBEAMSEARCH_H
#define ROBO_CRAFT_SEARCHALGORITHMBEAMSEARCH_H
#include "search/SearchAlgorithm.h"
namespace search
{
class SearchAlgorithmBeamSearch: public SearchAlgorithm
{
public:

    int maxLayerNodeExplore = 10000;

    SearchAlgorithmBeamSearch(std::shared_ptr<SearchGraph> graph,
                              int maxLayerNodeExplore):
          SearchAlgorithm(graph),
          maxLayerNodeExplore(maxLayerNodeExplore)
    {

    }

public:

    double search(AssemblySequence &sequence) override;

    double search(PtrS inputNode, AssemblySequence &sequence);

    void trimSolution();

};
}


#endif  //ROBO_CRAFT_SEARCHALGORITHMBEAMSEARCH_H
