//
// Created by Ziqi Wang on 02.03.2024.
//

#ifndef SCENE_H
#define SCENE_H
#include <memory>
#include "rigid_block/Assembly.h"
#include "robot/Robot.h"
#include "rigid_block/Analyzer.h"

namespace scene
{

    enum CollisionStatus {
        None = 0,
        RobotRobot = 1,
        RobotAssembly = 2,
        RobotGround= 3
    };

    class Scene
    {
    public:

        std::shared_ptr<rigid_block::Assembly> assembly_;

        std::vector<std::shared_ptr<robot::Robot> > robots_;

        std::shared_ptr<rigid_block::Analyzer> analyzer_;


    public:
        Scene(const std::shared_ptr<rigid_block::Assembly> &assembly, const std::vector<std::shared_ptr<robot::Robot> > &robots) {
            assembly_ = assembly;
            robots_ = robots;
            analyzer_ = assembly->createAnalyzer(false);
        }

    public:

        bool checkStability(const std::vector<int> &subset_part_ids,
                            const std::vector<int> &fixed_part_ids);

        CollisionStatus checkCollision(const std::vector<int> &subset_part_ids,
                                       const std::vector<int> &robot_ids,
                                       const std::vector<Eigen::VectorXd> &joint_angles);

        bool checkCollision(const std::vector<Eigen::MatrixXd> &Vs1,
                            const std::vector<Eigen::MatrixXd> &Vs2);

        bool checkCollisionWithGround(const std::vector<Eigen::MatrixXd> &Vs);
    };

}


#endif //SCENE_H
