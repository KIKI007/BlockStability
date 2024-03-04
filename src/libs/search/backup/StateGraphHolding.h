//
// Created by 汪子琦 on 04.09.22.
//

#ifndef ROBO_CRAFT_STATEGRAPHMULTIPLEARMS_H
#define ROBO_CRAFT_STATEGRAPHMULTIPLEARMS_H
#include "StateGraph.h"
#include "SearchGenerator.h"
namespace search
{

//mode
//0 uninstalled
//1 installed
//2 + robot_id, robot held

class StateGraphHolding : public StateGraph
{
public:

    bool solution_include_empty_node_ = true;

    std::vector<std::shared_ptr<StateGraph_BaseGenerator>> generators_;

public:

    StateGraphHolding(std::shared_ptr<PartGraph> partGraph);

public:

    std::string node_label(PtrN node) override;

    void createRootNode() override;

    bool checkEndNode(PtrN node) override;

    void getSolution(std::vector<PtrN> nodes, AssemblySequence &sequence) override;

    std::vector<unsigned> finishedAssemblyState() override;

    void to_json(nlohmann::json &result);

    void from_json(const nlohmann::json &input);

    std::shared_ptr<StateGraph_BaseGenerator> get_generator(std::string name);

public:

    void expandNodes(std::vector<PtrN> &input_nodes, std::vector<StateGraphEdge> &edgeItems) override;
};
}


#endif  //ROBO_CRAFT_STATEGRAPHMULTIPLEARMS_H
