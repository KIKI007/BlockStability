//
// Created by Ziqi Wang on 15.02.2024.
//

#ifndef ROBOT_H
#define ROBOT_H

#include "Eigen/Dense"
#include "tinyxml2.h"
#include "util/readOBJ.h"
#include "ikfast.h"
#include "util/Transform.h"
#include "collision.h"
#define IKFAST_HAS_LIBRARY
namespace robot
{
    struct RobotJoint;

    struct RobotLink
    {
        std::string name;
        Eigen::MatrixXd collision_meshV;
        Eigen::MatrixXi collision_meshF;

        Eigen::MatrixXd visual_meshV;
        Eigen::MatrixXi visual_meshF;

        util::Transform transf;
        std::weak_ptr<RobotJoint> parent_joint;
        std::weak_ptr<RobotJoint> child_joint;

    };

    struct RobotJoint
    {
        std::string name;
        util::Transform transf;
        Eigen::Vector3d axis_xyz;
        std::weak_ptr<RobotLink> parent_link;
        std::weak_ptr<RobotLink> child_link;
        int state_id;
    };

    class Robot
    {
    public:

        util::Transform baseT_, eeT_;

        Eigen::MatrixXd joint_range_;

        std::string name_;

    public:

        std::vector<std::shared_ptr<RobotLink>> links_;

        std::vector<std::shared_ptr<RobotJoint>> joints_;

        std::weak_ptr<RobotLink> root_, EF_;

    public:

        Robot(std::string name);

    public:

        void forward(const Eigen::VectorXd &j,
                     std::vector<Eigen::Matrix4d> &jointT,
                     std::vector<Eigen::Matrix4d> &linkT);

        void load(const std::string &folder_name, tinyxml2::XMLDocument &robot_urdf);

        std::shared_ptr<RobotLink> findLink(std::string name);

    public:

        util::Transform forwardEE(const Eigen::VectorXd &j);

        std::vector<Eigen::VectorXd> inverseEE(const util::Transform &transf);

        std::vector<Eigen::MatrixXd> computeGeometry(const Eigen::VectorXd &j, bool visual = true);
    };

}



#endif //ROBOT_H
