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

namespace search
{

    class SearchGraph_BaseGenerator : std::enable_shared_from_this<StateGraph_BaseGenerator> {
    public:
        std::string label_;

        StateGraph *graph_;

        std::vector<std::shared_ptr<StateGraph_BaseConstraint> > constraints_;

    public:
        SearchGraph_BaseGenerator(StateGraph *graph) {
            label_ = "base_generator";
            graph_ = graph;
        }

        SearchGraph_BaseGenerator() {
            label_ = "base_generator";
        }

        void expandNodes(std::vector<StateGraph::PtrN> &input_nodes,
                         std::vector<StateGraphEdge> &edges);

        virtual void newNodes(StateGraph::PtrN node, std::vector<std::vector<unsigned> > &assemblyStates) = 0;

        void trimNodes(const std::vector<StateGraph::PtrN> &prev_nodes,
                       const std::vector<std::vector<unsigned> > &next_states,
                       std::vector<StateGraphEdge> &edges);

        void checkNodes(const std::vector<std::vector<unsigned> > &prev_states,
                        const std::vector<std::vector<unsigned> > &next_states,
                        std::vector<bool> &checked_status);

        void addBaseOrderConstraints();

        void from_json(nlohmann::json input);

        nlohmann::json to_json();
    };


    class StateGraph_RobotGenerator : public StateGraph_BaseGenerator
    {
    public:
        int robotID_ = -1;
    public:
        StateGraph_RobotGenerator(StateGraph *graph): StateGraph_BaseGenerator(graph)
        {
            label_ = "robot_generator";
        }

        void newNodes(StateGraph::PtrN node, std::vector<std::vector<unsigned> > &assemblyStates) override;

        int generator_mode(){return robotID_ + 2;}
    };

    class StateGraph_GeneratorFactory
    {
    public:
        static std::shared_ptr<StateGraph_BaseGenerator> from_json(nlohmann::json input, StateGraph *graph);

        static std::shared_ptr<StateGraph_BaseGenerator> create(std::string label);
    };
}
#endif  //CMAKELISTS_TXT_STATEGRAPHGENERATOR_H
