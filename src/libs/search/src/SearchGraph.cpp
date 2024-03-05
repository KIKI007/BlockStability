//
// Created by Ziqi Wang on 04.03.2024.
//

#include "search/SearchGraph.h"

namespace search {
    SearchGraph::SearchGraph(std::shared_ptr<scene::Scene> scene) {
        scene_ = scene;
        std::shared_ptr<search::PartGraph> part_graph = std::make_shared<search::PartGraph>(scene->assembly_);
        state_graph_ = std::make_shared<search::StateGraphHelding>(part_graph);
        for(int id = 0; id < scene->assembly_->blocks_.size(); id++) {
            if(scene->assembly_->blocks_[id]->ground_)
                state_graph_->boundaryPartIDs_.push_back(id);
        }


        for(int id = 0; id < scene_->robots_.size(); id++)
        {
            auto generator = std::make_shared<search::SearchGraph_RobotGenerator>(state_graph_.get(), scene_.get());
            std::shared_ptr<search::SearchGraph_BaseConstraint> stability =
                std::make_shared<search::SearchGraph_StabilityConstraint>(state_graph_.get(), 1E-6);
            std::shared_ptr<search::SearchGraph_BaseConstraint> collision =
                   std::make_shared<search::SearchGraph_CollisonConstraint>(state_graph_.get(), scene_.get());
            generator->robotID_ = id;
            generator->constraints_.push_back(stability);
            generator->constraints_.push_back(collision);
            generators_.push_back(generator);
        }
    }

    PtrS SearchGraph::createRootNode()
    {
        state_graph_->clear();

        std::vector<unsigned> assembly_state;
        assembly_state.resize(state_graph_->nChunk_, 0);

        for(int id = 0; id < state_graph_->startPartIDs_.size(); id++){
            state_graph_->setPartMode(state_graph_->startPartIDs_[id], 1, assembly_state);
        }

        for(int id = 0; id < state_graph_->boundaryPartIDs_.size(); id++)
        {
            state_graph_->setPartMode(state_graph_->boundaryPartIDs_[id], 1, assembly_state);
        }

        state_graph_->addNewNode(assembly_state);
        state_graph_->nodes_[0]->currentCost = 0;

        PtrS node = std::make_shared<SearchNode>();
        node->assembly_state = assembly_state;
        node->prev_node = nullptr;
        return node;

    }

    bool SearchGraph::checkEndNode(PtrS node) {
        bool all_installed = (state_graph_->getInstalledParts(node->assembly_state).size() == state_graph_->endPartIDs_.size());
        bool all_robot_inactive = (state_graph_->getFixedParts(node->assembly_state).size() == 0);
        return all_installed && all_robot_inactive;
    }

    AssemblySequence SearchGraph::getSolution(const std::vector<PtrS> &input_nodes) {
        AssemblySequence sequence;

        if(!state_graph_->startPartIDs_.empty())
        {
            AssemblyStep step;
            step.installPartIDs_ = state_graph_->startPartIDs_;
            step.holdPartIDs_ = {};
            step.boundaryPartIDs_ = state_graph_->boundaryPartIDs_;
            step.robot_ids_ = {};
            step.robot_angles_ = {};
            sequence.steps.push_back(step);
        }

        for (int id = 1; id < input_nodes.size(); id++)
        {
            PtrS prevNode = input_nodes[id - 1];
            PtrS currNode = input_nodes[id];

            auto prevState = prevNode->assembly_state;
            auto currState = currNode->assembly_state;

            AssemblyStep step;
            step.installPartIDs_ = state_graph_->getNewInstalledParts(prevState, currState);
            step.boundaryPartIDs_ = state_graph_->boundaryPartIDs_;
            step.holdPartIDs_ = currNode->held_part_ids;
            step.robot_angles_ = currNode->robot_angles_;
            step.robot_ids_ = currNode->robot_ids;
            sequence.steps.push_back(step);
        }
        return sequence;
    }

    std::vector<PtrS> SearchGraph::expandNodes(const std::vector<PtrS> &input_nodes)
    {
        //human add bars
        std::vector<PtrS> output_nodes;
        for(auto generator: generators_)
        {
            std::vector<PtrS> tmp;
            tmp = generator->expandNodes(input_nodes);
            output_nodes.insert(output_nodes.end(), tmp.begin(), tmp.end());
        }
        return output_nodes;
    }
}

