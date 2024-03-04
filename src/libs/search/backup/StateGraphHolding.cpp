//
// Created by 汪子琦 on 04.09.22.
//

#include "search/StateGraphHolding.h"
#include "../include/search/SearchGenerator.h"

namespace search
{

StateGraphHolding::StateGraphHolding(std::shared_ptr<PartGraph> partGraph, int nRobot)
    : StateGraph(partGraph, nRobot + 2)
{
    nRobot_ = nRobot;
    std::vector<bool> parts_grounded = partGraph->computePartsGround();
    boundaryPartIDs_.clear();
    endPartIDs_.clear();
    for(int partID = 0; partID < partGraph->nNode(); partID++) {
        if(parts_grounded[partID])
            boundaryPartIDs_.push_back(partID);
        endPartIDs_.push_back(partID);
    }
    startPartIDs_ = boundaryPartIDs_;

    for(int iRobot = 0; iRobot < nRobot_; iRobot++)
    {
        std::shared_ptr<StateGraph_RobotGenerator> robot_generator = std::make_shared<StateGraph_RobotGenerator>(this);
        robot_generator->robotID_ = iRobot;
        std::shared_ptr<StateGraph_StabilityConstraint> constraint = std::make_shared<StateGraph_StabilityConstraint>(this, 1E-6);
        robot_generator->constraints_.push_back(constraint);
        generators_.push_back(robot_generator);
    }
}


std::string StateGraphHolding::node_label(PtrN node)
{
    std::vector<int> parts = getInstalledParts(node);
    std::string node_label = "\"[";
    for (int id = 0; id < parts.size(); id++)
    {
        node_label += std::to_string(parts[id]);
        if(std::find(boundaryPartIDs_.begin(), boundaryPartIDs_.end(), parts[id]) != boundaryPartIDs_.end()){
            node_label += "-";
        }
        if(computePartState(parts[id], *node) >= 2){
            node_label += "*";
        }
        if (id + 1 != parts.size()) {
            node_label += ", ";
        }
    }
    node_label += "]";
    node_label += "\"";
    return node_label;
}

void StateGraphHolding::getSolution(std::vector<PtrN> nodes, AssemblySequence &sequence)
{
    sequence.steps.clear();

    if(!startPartIDs_.empty())
    {
        AssemblyStep step;
        step.installPartIDs_ = startPartIDs_;
        step.holdPartIDs_ = {};
        step.boundaryPartIDs_ = boundaryPartIDs_;
        step.actors_ = {};
        sequence.steps.push_back(step);
    }

    for (int id = 1; id < nodes.size(); id++)
    {
        PtrN prevNode = nodes[id - 1];
        PtrN currNode = nodes[id];

        State prevState = getAssemblyState(*prevNode);
        State currState = getAssemblyState(*currNode);

        print(currState);

        std::vector<int> newInstalledParts = getNewInstalledParts(prevState, currState);

        std::vector<int> prev_fixed = getFixedParts(prevState);
        std::vector<int> curr_fixed = getFixedParts(currNode);

        AssemblyStep step;
        for (int jd = 0; jd < newInstalledParts.size(); jd++)
        {
            step.installPartIDs_.push_back(newInstalledParts[jd]);
        }

        for(int partID : curr_fixed) {
            unsigned part_state = computePartState(partID, *currNode);
            step.actors_.push_back(part_state - 2);
        }
        step.holdPartIDs_ = curr_fixed;
        step.boundaryPartIDs_ = boundaryPartIDs_;

        sequence.steps.push_back(step);
    }
}

void StateGraphHolding::expandNodes(std::vector<PtrN> &input_nodes,
                                    std::vector<StateGraphEdge> &edgeItems)
{
    //human add bars
    std::vector<StateGraphEdge> new_edge_items;
    std::vector<StateGraphEdge> exist_edge_items;
    for(auto generator: generators_)
    {
        std::vector<StateGraphEdge> edges;
        generator->expandNodes(input_nodes, edges);
        for(int id = 0; id < edges.size(); id++){
            State &next_state = edges[id].next_state;
            StateGraphEdge & edge = edges[id];
            auto [newNodeID, createNewNode] = addNewNode(next_state);
            edge.next_node = nodes_[newNodeID];
            edgeItems.push_back(edge);
        }
    }
}

void StateGraphHolding::createRootNode()
{
    clear();

    std::vector<unsigned> assemblyState;
    assemblyState.resize(nChunk_, 0);

    for(int id = 0; id < startPartIDs_.size(); id++){
        setPartMode(startPartIDs_[id], 1, assemblyState);
    }

    for(int id = 0; id < boundaryPartIDs_.size(); id++)
    {
        setPartMode(boundaryPartIDs_[id], 1, assemblyState);
    }

    addNewNode(assemblyState);
    nodes_[0]->currentCost = 0;

    nodeEdgeOffsetStart_.push_back(0);
    nodeEdgeOffsetEnd_.push_back(0);
}

bool StateGraphHolding::checkEndNode(search::StateGraph::PtrN node) {
    bool all_installed = (getInstalledParts(node).size() == endPartIDs_.size());
    bool all_robot_inactive = (getFixedParts(node).size() == 0);
    return all_installed && all_robot_inactive;
}

std::vector<unsigned> StateGraphHolding::finishedAssemblyState() {
    std::vector<unsigned> fullAssemblyState;
    fullAssemblyState.resize(nChunk_, 0);
    for(int id = 0 ;id < endPartIDs_.size(); id++){
        setPartMode(endPartIDs_[id], 1, fullAssemblyState);
    }
    return fullAssemblyState;
}

void StateGraphHolding::to_json(nlohmann::json &result){
    StateGraph::to_json(result);
    result["search"]["generator"] = {};
    for(int id = 0; id < generators_.size(); id++){
        result["search"]["generator"].push_back(generators_[id]->to_json());
    }
}

void StateGraphHolding::from_json(const nlohmann::json &input)
{
    StateGraph::from_json(input);
    for(int id = 0; id < input["search"]["generator"].size(); id++)
    {
        nlohmann::json generator_json = input["search"]["generator"][id];
        std::shared_ptr<StateGraph_BaseGenerator> generator = StateGraph_GeneratorFactory::from_json(generator_json, this);
        generators_.push_back(generator);
    }
}

std::shared_ptr<StateGraph_BaseGenerator> StateGraphHolding::get_generator(std::string name) {
    for(auto generator : generators_) {
        if(generator->label_ == name) {
            return generator;
        }
    }
    return nullptr;
}
}

