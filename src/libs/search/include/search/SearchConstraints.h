//
// Created by 汪子琦 on 01.11.23.
//

#ifndef CMAKELISTS_TXT_STATEGRAPHCONSTRAINTS_H
#define CMAKELISTS_TXT_STATEGRAPHCONSTRAINTS_H

#include <search/StateGraph.h>
#include <string>
#include <scene/Scene.h>

struct SearchNode
{
    std::vector<unsigned> assembly_state;

    std::vector<int> robot_ids;
    std::vector<Eigen::VectorXd> robot_angles_;
    std::vector<int> held_part_ids;

    enum ActionType {
        Install = 0,
        Release = 1
    } action_type;

    std::shared_ptr<SearchNode> prev_node;
};

typedef std::shared_ptr<SearchNode> PtrS;

namespace search
{

    class SearchGraph_BaseConstraint : std::enable_shared_from_this<SearchGraph_BaseConstraint> {
    public:

        StateGraph *graph_;

        SearchGraph_BaseConstraint(StateGraph *graph) {
            graph_ = graph;
        }

    public:

        virtual bool check(const PtrS &prev_node, const PtrS &curr_node) { return false; }

        //debug
        void print(StateGraph::State state);
    };


    class SearchGraph_StabilityConstraint : public SearchGraph_BaseConstraint
    {

    public:
        double max_deformation_;

    public:
        SearchGraph_StabilityConstraint(StateGraph *graph, double max_deformation):
        SearchGraph_BaseConstraint(graph)
        {
            max_deformation_ = max_deformation;
        }

    public:
        bool check(const PtrS &prev_node, const PtrS &curr_node) override;
    };

    class SearchGraph_CollisonConstraint : public SearchGraph_BaseConstraint {
    public:
        scene::Scene *scene_;

    public:
        SearchGraph_CollisonConstraint(StateGraph *graph, scene::Scene *scene):
        SearchGraph_BaseConstraint(graph)
        {
            scene_ = scene;
        }

    public:
        bool check(const PtrS &prev_node, const PtrS &curr_node) override;

    };

} // namespace search

#endif  //CMAKELISTS_TXT_STATEGRAPHCONSTRAINTS_H
