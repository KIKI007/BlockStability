//
// Created by 汪子琦 on 01.11.23.
//

#include "search/SearchGenerator.h"

namespace search
{
    std::vector<PtrS> SearchGraph_BaseGenerator::expandNodes(const std::vector<PtrS> &input_nodes)
    {
        tbb::concurrent_vector<std::pair<PtrS, PtrS> > result_pairs;

        for (int id = 0; id < input_nodes.size(); id++) {
            PtrS input_node = input_nodes[id];
            std::vector<PtrS> tmp;
            tmp = generateNewNodes(input_node);
            for (int jd = 0; jd < tmp.size(); jd++) {
                result_pairs.push_back({input_node, tmp[jd]});
            }
        }

        std::vector<PtrS> prev_nodes, next_nodes;

        for (int id = 0; id < result_pairs.size(); id++) {
            prev_nodes.push_back(result_pairs[id].first);
            next_nodes.push_back(result_pairs[id].second);
        }

        trimNodes(prev_nodes, next_nodes);

        return next_nodes;
    }

    void SearchGraph_BaseGenerator::trimNodes(std::vector<PtrS> &prev_nodes,
                                              std::vector<PtrS> &curr_nodes) {
        std::vector<PtrS> new_prev_nodes;
        std::vector<PtrS> new_curr_nodes;

        for (int id = 0; id < curr_nodes.size(); ++id)
        {
            PtrS curr_node = curr_nodes[id];
            PtrS prev_node = prev_nodes[id];

            bool violate = false;
            StateGraphEdge edge;

            for (auto constraint: constraints_)
            {
                bool check_result = constraint->check(prev_node, curr_node);
                if (check_result == false) {
                    violate = true;
                    break;
                }
            }

            if (!violate)
            {
                new_curr_nodes.push_back(curr_node);
                new_prev_nodes.push_back(prev_node);
            }
        }

        curr_nodes = new_curr_nodes;
        prev_nodes = new_prev_nodes;
    }

    std::vector<PtrS> SearchGraph_RobotGenerator::generateNewNodes(PtrS input_node)
    {

        //check status
        int held_part_id = -1;
        int index;
        for(int id = 0; id < input_node->robot_ids.size(); id++)
        {
            if(input_node->robot_ids[id] == robotID_) {
                held_part_id = input_node->held_part_ids[id];
                index = id;
                break;
            }
        }

        std::vector<PtrS> nodes;

        if (held_part_id != -1)
        {
            std::vector<unsigned> new_state = input_node->assembly_state;
            graph_->setPartMode(held_part_id, 1, new_state);
            PtrS new_node = std::make_shared<SearchNode>();

            new_node->assembly_state = new_state;

            //copy
            new_node->robot_ids = input_node->robot_ids;
            new_node->held_part_ids = input_node->held_part_ids;
            new_node->robot_angles_ = input_node->robot_angles_;

            //release
            new_node->robot_ids.erase(new_node->robot_ids.begin() + index);
            new_node->held_part_ids.erase(new_node->held_part_ids.begin() + index);
            new_node->robot_angles_.erase(new_node->robot_angles_.begin() + index);

            //prev
            new_node->prev_node = input_node;
            new_node->action_type = SearchNode::ActionType::Release;
            nodes.push_back(new_node);
        }
        else
        {
            std::vector<int> installed_parts = graph_->getInstalledParts(input_node->assembly_state);
            std::vector<int> neighbour_parts;
            graph_->partGraph_->computeNodesNeighbour(installed_parts, neighbour_parts, 1);

            // but if not, robot can add bars
            for (int select_part_id: neighbour_parts)
            {
                if (graph_->computePartState(select_part_id, input_node->assembly_state) == 0)
                {
                    std::vector<unsigned> new_state = input_node->assembly_state;
                    graph_->setPartMode(select_part_id, 2, new_state);
                    auto block = scene_->assembly_->blocks_[select_part_id];
                    auto r = scene_->robots_[robotID_];
                    auto eelist = block->eeAnchor();
                    for(auto &ee: eelist)
                    {
                        auto js = r->inverseEE(ee);
                        for(auto &j : js)
                        {
                            PtrS new_node = std::make_shared<SearchNode>();
                            new_node->assembly_state = new_state;

                            new_node->robot_ids = input_node->robot_ids;
                            new_node->held_part_ids = input_node->held_part_ids;
                            new_node->robot_angles_ = input_node->robot_angles_;

                            //install
                            new_node->robot_ids.push_back(robotID_);
                            new_node->held_part_ids.push_back(select_part_id);
                            new_node->robot_angles_.push_back(j);

                            new_node->prev_node = input_node;
                            new_node->action_type = SearchNode::ActionType::Install;
                            nodes.push_back(new_node);
                        }
                    }
                }
            }
        }
        return nodes;
    }
} // namespace search
