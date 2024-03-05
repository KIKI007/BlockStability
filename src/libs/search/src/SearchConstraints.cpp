//
// Created by 汪子琦 on 01.11.23.
//
#include "search/SearchConstraints.h"


bool search::SearchGraph_StabilityConstraint::check(const PtrS &prev_node, const PtrS &curr_node)
{
    if(curr_node->action_type == SearchNode::Release) {
        std::vector<int> installed_part_ids = graph_->getInstalledParts(curr_node->assembly_state);
        std::vector<int> fixed_part_ids = graph_->getFixedParts(curr_node->assembly_state);

        for(auto partID: graph_->boundaryPartIDs_) {
            if(std::find(fixed_part_ids.begin(), fixed_part_ids.end(), partID) == fixed_part_ids.end()) {
                fixed_part_ids.push_back(partID);
            }
        }

        auto [newNodeID, createNewNode] = graph_->addNewNode(curr_node->assembly_state);

        if(createNewNode) {
            graph_->nodes_[newNodeID]->currentCost
            = graph_->partGraph_->evaluateStability(installed_part_ids, fixed_part_ids);
        }
        double stability = graph_->nodes_[newNodeID]->currentCost;

        if (stability > max_deformation_) {
            return false;
        }
    }
    return true;
}

bool search::SearchGraph_CollisonConstraint::check(const PtrS &prev_node, const PtrS &curr_node)
{
    if(curr_node->action_type == SearchNode::Install)
    {
        std::vector<int> installed_part_ids = graph_->getInstalledParts(curr_node->assembly_state);
        bool collision = scene_->checkCollision(installed_part_ids, curr_node->robot_ids, curr_node->robot_angles_);
        if(collision) return false;
    }
    return true;
}

//for(int id = 0; id < scene_->robots_.size(); id++) {
    // int robot_id = curr_node->robot_ids[id];
    // int block_id = curr_node->robot_held_part_ids[id];
    // int ee_id = curr_node->robot_help_part_ee_inds_[id];
    // auto r = scene_->robots_[robot_id];
    // auto block = scene_->assembly_->blocks_[block_id];
    // auto ee = block->eeAnchor()[ee_id];
    // auto js = r->inverseEE(ee);
    // auto j = js[]
//}