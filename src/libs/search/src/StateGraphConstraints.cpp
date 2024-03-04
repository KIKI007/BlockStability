//
// Created by 汪子琦 on 01.11.23.
//
#include "search/SearchConstraints.h"

bool search::StateGraph_BaseConstraint::compute_current_beam(const StateGraph::State& prev_node, const StateGraph::State& curr_node, int &curr_beam)
{
    std::vector<int> newInstalledParts = graph_->getNewInstalledParts(prev_node, curr_node);
    if (newInstalledParts.size() > 1 || newInstalledParts.size() == 0)
        return false;
    curr_beam = newInstalledParts.front();
    return true;
}

void search::StateGraph_BaseConstraint::from_json(nlohmann::json input) {

}

nlohmann::json search::StateGraph_BaseConstraint::to_json() {
    nlohmann::json output;
    output["label"] = label_;
    return output;
}

void search::StateGraph_BaseConstraint::print(search::StateGraph::State state)
{
    std::vector<int> installed_beams = graph_->getInstalledParts(state);
    std::vector<int> fixed_beams = graph_->getFixedParts(state);

    for(int id = 0; id < installed_beams.size(); id++){
        int part_id = installed_beams[id];
        std::cout << part_id;
        if(std::find(fixed_beams.begin(), fixed_beams.end(), part_id) != fixed_beams.end()){
            std::cout << "*";
        }
        std::cout << " ";
    }
    std::cout << std::endl;
}

bool search::StateGraph_PartialOrderConstraint::check(const search::StateGraph::State &prev_node,
                                                      const search::StateGraph::State &curr_node,
                                                      double &cost) {
    if(graph_->computePartState(order_.y(), curr_node) > 0){
        if(graph_->computePartState(order_.x(), curr_node) == 0){
            return false;
        }
    }
    return true;
}

void search::StateGraph_PartialOrderConstraint::from_json(nlohmann::json input) {
    std::vector<int> order = input["order"].get<std::vector<int>>();
    if(!order.empty()){
        order_.x() = order.front();
        order_.y() = order.back();
    }
}
nlohmann::json search::StateGraph_PartialOrderConstraint::to_json() {
    nlohmann::json output;
    output["label"] = label_;
    output["order"] = {order_.x(), order_.y()};
    return output;
}

bool search::StateGraph_StabilityConstraint::check(const search::StateGraph::State & prev_state,
                                                   const search::StateGraph::State & curr_state,
                                                   double &cost)
{
    std::vector<int> curr_installedBeams = graph_->getInstalledParts(curr_state);
    std::vector<int> curr_fixedBeams = graph_->getFixedParts(curr_state);

    std::vector<int> fixed_partIDs = graph_->boundaryPartIDs_;
    fixed_partIDs.insert(fixed_partIDs.end(), curr_fixedBeams.begin(), curr_fixedBeams.end());

    cost = graph_->partGraph_->evaluateStability(curr_installedBeams, fixed_partIDs);
    if (cost > max_deformation_) {
        return false;
    }
    return true;
}

void search::StateGraph_StabilityConstraint::from_json(nlohmann::json input) {
    max_deformation_ = input["max_deformation"].get<double>();
}

nlohmann::json search::StateGraph_StabilityConstraint::to_json() {
    nlohmann::json output;
    output["label"] = label_;
    output["max_deformation"] = max_deformation_;
    return output;
}

std::shared_ptr<search::StateGraph_BaseConstraint> search::StateGraph_ConstraintFactory::from_json(nlohmann::json input, StateGraph*graph) {
    std::string label = input["label"];
    std::shared_ptr<StateGraph_BaseConstraint> constraint;

    if(label == "partial_order_constraint"){
        constraint = std::make_shared<StateGraph_PartialOrderConstraint>(graph);
    }
    else if(label == "stability_constraint"){
        constraint = std::make_shared<StateGraph_StabilityConstraint>(graph);
    }

    if(constraint)
        constraint->from_json(input);

    return constraint;
}