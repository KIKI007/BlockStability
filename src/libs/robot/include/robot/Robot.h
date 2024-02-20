//
// Created by Ziqi Wang on 15.02.2024.
//

#ifndef ROBOT_H
#define ROBOT_H

#include "Eigen/Dense"
#include "tinyxml2.h"
#include "util.h"
#include "ikfast.h"
#define IKFAST_HAS_LIBRARY

namespace robot
{
    struct Transformation
    {
        Eigen::Vector3d rpy = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d xyz = Eigen::Vector3d(0, 0, 0);

        Eigen::Matrix4d mat() const;

        Eigen::Matrix3d rot() const;

        void from_rot(const Eigen::Matrix3d &rot);

        Transformation(){}
        Transformation(tinyxml2::XMLElement *node);
    };

    struct RobotJoint;

    struct RobotLink
    {
        std::string name;
        Eigen::VectorXd collision_meshV;
        Eigen::VectorXi collision_meshF;
        Eigen::MatrixXd visual_meshV;
        Eigen::MatrixXi visual_meshF;

        Transformation transf;
        std::weak_ptr<RobotJoint> parent_joint;
        std::weak_ptr<RobotJoint> child_joint;

    };

    struct RobotJoint
    {
        std::string name;
        Transformation transf;
        Eigen::Vector3d axis_xyz;
        std::weak_ptr<RobotLink> parent_link;
        std::weak_ptr<RobotLink> child_link;
        int state_id;
    };

    class Robot
    {
    public:

        Transformation baseT_, eeT_;

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

        Transformation forwardEE(const Eigen::VectorXd &j);

        std::vector<Eigen::VectorXd> inverseEE(const Transformation &transf);
    };

}



#endif //ROBOT_H
