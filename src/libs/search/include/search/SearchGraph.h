//
// Created by Ziqi Wang on 04.03.2024.
//

#ifndef SEARCHGRAPH_H
#define SEARCHGRAPH_H

#include "StateGraph.h"
#include "SearchConstraints.h"
#include "SearchGenerator.h"
#include "scene/Scene.h"
#include "PartGraph.h"
#include "AssemblySequence.h"

namespace search {
    class SearchGraph {
    public:
        std::vector<std::shared_ptr<search::SearchGraph_BaseGenerator>> generators_;
        std::shared_ptr<search::StateGraph> state_graph_;
        std::shared_ptr<scene::Scene> scene_;

    public:

        SearchGraph(std::shared_ptr<scene::Scene> scene);

    public:

        PtrS createRootNode();

        bool checkEndNode(PtrS node);

        AssemblySequence getSolution(const std::vector<PtrS> &input_nodes);

        std::vector<PtrS> expandNodes(const std::vector<PtrS> &input_nodes);

    };
}

#endif //SEARCHGRAPH_H
