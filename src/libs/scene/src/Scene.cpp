//
// Created by Ziqi Wang on 02.03.2024.
//

#include "scene/Scene.h"

#include "rigid_block/Assembly.h"

namespace scene {
    bool Scene::checkStability(
        const std::vector<int> &subset_part_ids,
        const std::vector<int> &fixed_part_ids)
    {
        std::vector<rigid_block::Analyzer::PartStatus> status;
        status.resize(assembly_->blocks_.size(), rigid_block::Analyzer::Uninstalled);

        // installed parts
        for (int partID: subset_part_ids)
        {
            status[partID] = rigid_block::Analyzer::Installed;
        }

        //boundary + fixed
        for(int partID = 0; partID < status.size(); partID++) {
            if(assembly_->blocks_[partID]->ground_) {
                status[partID] = rigid_block::Analyzer::Fixed;
            }
        }
        for (int partID: fixed_part_ids) {
            status[partID] = rigid_block::Analyzer::Fixed;
        }

        rigid_block::AnalysisResult result;
        return analyzer_->solve(status, result);
    }

    CollisionStatus Scene::checkCollision(
        const std::vector<int> &subset_part_ids,
        const std::vector<int> &robot_ids,
        const std::vector<Eigen::VectorXd> &joint_angles)
    {
        std::vector<Eigen::MatrixXd> partVs;
        std::vector<std::vector<Eigen::MatrixXd>> robotVs;

        for(int partID : subset_part_ids) {
            partVs.push_back(assembly_->blocks_[partID]->V_);
        }

        for(int ir = 0; ir < robot_ids.size(); ir++) {
            int actorID = robot_ids[ir];
            auto Vs = robots_[actorID]->computeGeometry(joint_angles[ir], false);
            robotVs.push_back(Vs);
        }

        //among robots
        for(int id = 0; id < robotVs.size(); id++) {
            for(int jd = id + 1; jd < robotVs.size(); jd++) {
                if(checkCollision(robotVs[id], robotVs[jd])) {
                    return RobotRobot;
                }
            }
        }

        //among robot and blocks
        for(int id = 0; id < robotVs.size(); id++)
        {
            if(checkCollision(robotVs[id], partVs))
            {
                return RobotAssembly;
            }
        }

        //among robots and ground
        for(int id = 0; id < robotVs.size(); id++) {
            if(checkCollisionWithGround(robotVs[id])) {
                return RobotGround;
            }
        }

        return None;
    }

    bool Scene::checkCollision(const std::vector<Eigen::MatrixXd> &Vs1, const std::vector<Eigen::MatrixXd> &Vs2) {
        for(auto &V1 : Vs1)
        {
            for(auto &V2: Vs2)
            {
                if(robot::checkCollision(V1, V2)) {
                    return true;
                }
            }
        }
        return false;
    }

    //ingore the first object
    bool Scene::checkCollisionWithGround(const std::vector<Eigen::MatrixXd> &Vs) {
        for(int id = 1; id < Vs.size(); id++) {
            if(robot::checkCollisionWithGround(Vs[id])) {
                return true;
            }
        }
        return false;
    }
}
