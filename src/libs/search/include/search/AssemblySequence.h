//
// Created by 汪子琦 on 17.03.22.
//

#ifndef ROBO_CRAFT_ASSEMBLYSEQUENCE_H
#define ROBO_CRAFT_ASSEMBLYSEQUENCE_H

#include <vector>
#include "nlohmann/json.hpp"
#include <fstream>
#include <Eigen/Dense>

namespace search
{
struct AssemblyStep {
public:
    std::vector<int> installPartIDs_;
    std::vector<int> holdPartIDs_;
    std::vector<int> boundaryPartIDs_;
    std::vector<int> robot_ids_;
    std::vector<Eigen::VectorXd> robot_angles_;

public:
//    std::vector<Eigen::Vector3d> partDrts;
//    std::vector<int> deformPartIDs;
//    std::vector<std::vector<Eigen::Vector3d>> deformLines;
//    std::vector<std::vector<double>> deformValues;
//    double compliance = 0;
};

class AssemblySequence {
public:
    std::vector<AssemblyStep> steps;

public:

    void loadFromFile(std::string filename)
    {
        std::ifstream fin(filename);
        nlohmann::json json_content;
        fin >> json_content;
        loadFromJson(json_content);
        fin.close();
    }

    void loadFromJson(nlohmann::json json_node)
    {
        steps.clear();
        for(int id = 0; id < json_node["assembly_sequence"].size(); id++)
        {
            nlohmann::json step_node = json_node["assembly_sequence"][id];
            AssemblyStep step;
            step.installPartIDs_ = step_node["installPartIDs"].get<std::vector<int>>();
            step.holdPartIDs_ = step_node["holdPartIDs"].get<std::vector<int>>();
            steps.push_back(step);
        }
    }

    void writeToJson(nlohmann::json &json_node)
    {
        nlohmann::json assembly_node = nlohmann::json::array();
        for (int id = 0; id < steps.size(); id++)
        {
            nlohmann::json step_node;
            step_node["installPartIDs"] = steps[id].installPartIDs_;
            step_node["holdPartIDs"] = steps[id].holdPartIDs_;
            assembly_node.push_back(step_node);
        }
        json_node["assembly_sequence"] = assembly_node;
    }
};
}
#endif  //ROBO_CRAFT_ASSEMBLYSEQUENCE_H
