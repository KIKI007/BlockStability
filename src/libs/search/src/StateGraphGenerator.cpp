//
// Created by 汪子琦 on 01.11.23.
//

#include "search/StateGraphGenerator.h"


namespace search
{
    void StateGraph_BaseGenerator::expandNodes(std::vector<StateGraph::PtrN> &input_nodes,
                                               std::vector<StateGraphEdge> &edges) {
        tbb::concurrent_vector<std::pair<std::vector<unsigned int>, StateGraph::PtrN> > results_tbb;

        //tbb::parallel_for(tbb::blocked_range<int>(0, input_nodes.size()), [&](tbb::blocked_range<int> r) {
        for(int id = 0; id < input_nodes.size(); id++){
            //for (int id = r.begin(); id < r.end(); ++id) {
            StateGraph::PtrN input_node = input_nodes[id];
            std::vector<std::vector<unsigned int> > tmp;
            newNodes(input_node, tmp);
            for (int jd = 0; jd < tmp.size(); jd++) {
                results_tbb.push_back({tmp[jd], input_node});
            }
        }
        //});

        std::vector<StateGraph::State> next_states;
        std::vector<StateGraph::PtrN> prev_nodes;

        for (int id = 0; id < results_tbb.size(); id++) {
            next_states.push_back(results_tbb[id].first);
            prev_nodes.push_back(results_tbb[id].second);
        }

        trimNodes(prev_nodes, next_states, edges);
    }

    void StateGraph_BaseGenerator::trimNodes(const std::vector<StateGraph::PtrN> &prev_nodes,
                                             const std::vector<StateGraph::State> &next_states,
                                             std::vector<StateGraphEdge> &edges)
    {
        tbb::concurrent_vector<StateGraphEdge> trimed_edges_tbb;

        //tbb::parallel_for(tbb::blocked_range<int>(0, next_states.size()), [&](tbb::blocked_range<int> r) {
            //for (int id = r.begin(); id < r.end(); ++id)
            for(int id = 0; id < next_states.size(); ++id)
            {
                StateGraph::State next_state = next_states[id];
                StateGraph::State prev_state = graph_->getAssemblyState(*prev_nodes[id]);

                bool violate = false;
                StateGraphEdge edge;
                edge.label = label_;
                edge.edge_cost = 1;

                for (auto constraint: constraints_) {
                    double cost = 0;
                    bool check_result = constraint->check(prev_state, next_state, cost);
                    edge.constraints_label.push_back(constraint->label_);
                    edge.constraints_cost.push_back(cost);
                    if (check_result == false) {
                        violate = true;
                        break;
                    }
                }

                if (!violate) {
                    edge.next_state = next_state;
                    edge.prev_node = prev_nodes[id];
                    trimed_edges_tbb.push_back(edge);
                }
            }
       // });

        for (int id = 0; id < trimed_edges_tbb.size(); id++) {
            StateGraphEdge edge = trimed_edges_tbb[id];
            edges.push_back(edge);
        }
    }

    void StateGraph_BaseGenerator::from_json(nlohmann::json input) {
        for (int id = 0; id < input["constraints"].size(); id++) {
            std::shared_ptr<StateGraph_BaseConstraint> constraint = StateGraph_ConstraintFactory::from_json(
                input["constraints"][id], graph_);
            constraints_.push_back(constraint);
        }
    }

    nlohmann::json StateGraph_BaseGenerator::to_json() {
        nlohmann::json output;
        output["label"] = label_;
        output["constraints"] = {};
        for (int id = 0; id < constraints_.size(); id++) {
            nlohmann::json node_output;
            node_output = constraints_[id]->to_json();
            output["constraints"].push_back(node_output);
        }
        return output;
    }

    void StateGraph_BaseGenerator::addBaseOrderConstraints()
    {
        std::vector<bool> ground_bars = graph_->partGraph_->computePartsGround();
        for (int id = 0; id < graph_->nPart_; id++) {
            int partI = id;
            if (ground_bars[partI] == false)
                continue;
            std::vector<unsigned> neighbour_parts = graph_->partGraph_->computeNodeNeighbour(partI);
            for (int partJ: neighbour_parts) {
                if (ground_bars[partJ] == false) {
                    std::shared_ptr<StateGraph_BaseConstraint> base_order_constraint
                    = std::make_shared<StateGraph_PartialOrderConstraint>(graph_, Eigen::Vector2i(partI, partJ));
                    constraints_.push_back(base_order_constraint);
                }
            }
        }
    }

    void StateGraph_RobotGenerator::newNodes(StateGraph::PtrN node,
                                             std::vector<std::vector<unsigned int> > &assemblyStates)
    {
        std::vector<unsigned> modes;
        bool inuse = false;
        int held_partID = -1;
        std::vector<int> installed_parts = graph_->getInstalledParts(node);
        std::vector<int> neighbour_parts;
        graph_->partGraph_->computeNodesNeighbour(installed_parts, neighbour_parts, 1);
        //for(int id = 0; id < graph_->nPart_; id++) neighbour_parts.push_back(id);

        for(int partID = 0; partID < graph_->nPart_; partID ++) {
            unsigned part_mode = graph_->computePartState(partID, *node);
            modes.push_back(part_mode);
            if(part_mode == generator_mode()) {
                inuse = true;
                held_partID = partID;
            }
        }

        std::vector<unsigned> states = graph_->getAssemblyState(*node);
        if(inuse){
            std::vector<unsigned> new_states = states;
            graph_->setPartMode(held_partID, 1, new_states);
            assemblyStates.push_back(new_states);
        }
        else{
            // but if not, robot can add bars
            for (int select_part_id: neighbour_parts)
            {
                if(modes[select_part_id] == 0)
                {
                    std::vector<unsigned> new_state = states;
                    graph_->setPartMode(select_part_id, generator_mode(), new_state);
                    assemblyStates.push_back(new_state);
                }
            }
        }
    }

    std::shared_ptr<StateGraph_BaseGenerator> StateGraph_GeneratorFactory::create(std::string label)
    {
        std::shared_ptr<StateGraph_BaseGenerator> base_generator;
        if (label == "robot_generator" || label == "robot") {
            base_generator = std::make_shared<StateGraph_RobotGenerator>(nullptr);
        }
        return base_generator;
    }


    std::shared_ptr<StateGraph_BaseGenerator> StateGraph_GeneratorFactory::from_json(nlohmann::json input,
        StateGraph *graph) {
        std::shared_ptr<StateGraph_BaseGenerator> base_generator;
        if (input.contains("label")) {
            if (input["label"] == "robot_generator") {
                base_generator = std::make_shared<StateGraph_RobotGenerator>(graph);
                base_generator->from_json(input);
            }
        }
        return base_generator;
    }
} // namespace search
