//
// Created by 汪子琦 on 01.11.23.
//

#ifndef CMAKELISTS_TXT_STATEGRAPHCONSTRAINTS_H
#define CMAKELISTS_TXT_STATEGRAPHCONSTRAINTS_H
#include <search/StateGraph.h>
#include <string>
#include <search/SubsetTree.h>
namespace search
{

class StateGraph_BaseConstraint: std::enable_shared_from_this<StateGraph_BaseConstraint> {
public:
    StateGraph* graph_;
    std::string label_;

public:
    StateGraph_BaseConstraint(StateGraph* graph) {
        graph_ = graph;
        label_ = "base_constraint";
    }

public:
    virtual bool check(const StateGraph::State &prev_node, const StateGraph::State &curr_node, double &cost) {
        cost = 0;
        return false;
    }

    bool compute_current_beam(const StateGraph::State &prev_node, const StateGraph::State &curr_node, int &curr_beam);

    //debug
    void print(StateGraph::State state);

public:

    virtual void from_json(nlohmann::json input);

    virtual nlohmann::json to_json();
};

class StateGraph_PartialOrderConstraint: public StateGraph_BaseConstraint{
public:
    Eigen::Vector2i order_; //order[0] < order[1]

    StateGraph_PartialOrderConstraint(StateGraph* graph) : StateGraph_BaseConstraint(graph) {
        label_ = "partial_order_constraint";
    }

    StateGraph_PartialOrderConstraint(StateGraph* graph, Eigen::Vector2i order) : StateGraph_BaseConstraint(graph){
        label_ = "partial_order_constraint";
        order_ = order;
    }

    bool check(const StateGraph::State &prev_node, const StateGraph::State &curr_node, double &cost) override;

public:
    void from_json(nlohmann::json input) override;

    nlohmann::json to_json() override;
};

class StateGraph_StabilityConstraint : public StateGraph_BaseConstraint {

public:
    double max_deformation_;

    public:

    StateGraph_StabilityConstraint(StateGraph* graph)
        : StateGraph_BaseConstraint(graph)
    {
        label_ = "stability_constraint";
    }

    StateGraph_StabilityConstraint(StateGraph* graph, double max_deformation)
        : StateGraph_BaseConstraint(graph)
    {
        label_ = "stability_constraint";
        max_deformation_ = max_deformation;
    }

public:

    bool check(const StateGraph::State &prev_node, const StateGraph::State &curr_node, double &cost) override;

public:

    void from_json(nlohmann::json input) override;

    nlohmann::json to_json() override;
};

class StateGraph_ConstraintFactory{
public:

    static std::shared_ptr<StateGraph_BaseConstraint> from_json(nlohmann::json input, StateGraph* graph);
};

}  // namespace search

#endif  //CMAKELISTS_TXT_STATEGRAPHCONSTRAINTS_H
