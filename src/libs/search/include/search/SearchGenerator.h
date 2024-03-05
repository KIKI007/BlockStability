//
// Created by 汪子琦 on 01.11.23.
//

#ifndef CMAKELISTS_TXT_STATEGRAPHGENERATOR_H
#define CMAKELISTS_TXT_STATEGRAPHGENERATOR_H

#include <string>
#include "StateGraph.h"
#include "SearchConstraints.h"
#include "tbb/concurrent_vector.h"
#include "tbb/parallel_for.h"
#include "scene/Scene.h"

namespace search
{
    class SearchGraph_BaseGenerator : std::enable_shared_from_this<SearchGraph_BaseGenerator> {
    public:

        StateGraph *graph_;

        scene::Scene * scene_;

        std::vector<std::shared_ptr<SearchGraph_BaseConstraint> > constraints_;

    public:
        SearchGraph_BaseGenerator(StateGraph *graph, scene::Scene *scene) {
            graph_ = graph;
            scene_ = scene;
        }

        std::vector<PtrS> expandNodes(const std::vector<PtrS> &input_nodes);

        virtual std::vector<PtrS> generateNewNodes(PtrS node) = 0;

        void trimNodes(std::vector<PtrS> &prev_nodes, std::vector<PtrS> &curr_nodes);

        std::vector<bool>  checkNodes(const std::vector<PtrS> &prev_nodes,
                                      const std::vector<PtrS> &curr_nodes);
    };


    class SearchGraph_RobotGenerator : public SearchGraph_BaseGenerator
    {
    public:
        int robotID_ = -1;

        SearchGraph_RobotGenerator(StateGraph *graph, scene::Scene *scene): SearchGraph_BaseGenerator(graph, scene) {
            robotID_ = -1;
        }

        std::vector<PtrS> generateNewNodes(PtrS node) override;

    };
}
#endif  //CMAKELISTS_TXT_STATEGRAPHGENERATOR_H
