//
// Created by Ziqi Wang on 19.02.2024.
//

#ifndef ROBOTRENDER_H
#define ROBOTRENDER_H
#include <Eigen/Dense>
#include "robot/Robot.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
static int robot_count = 0;
namespace render {
    class RobotRender
    {
    public:
        bool show_collision_ = false;

    public:

        Eigen::VectorXf j_;

        Eigen::Vector3f ee_xyz_;

        Eigen::Vector3f ee_rpy_;

        int iksol_ = 2;

        int eeindex_ = 0;

        std::shared_ptr<robot::Robot> robot_;

        int robot_id_ = 0;

    public:

        std::vector<util::Transform> eelist_;

    public:

        std::vector<polyscope::SurfaceMesh *> robot_body_;

        std::vector<polyscope::CurveNetwork *> robot_ee_;

        polyscope::Group *robot_group_, *ee_group;


    private:

        const double ee_arrow_length_ = 0.1;

        const double ee_point_radius_ = 0.02;

        const double ee_arrow_radius_ = 0.01;


    public:

        RobotRender(std::string name, Eigen::Vector3d base_pos);

        ~RobotRender();

    public:

        void gui();

        void update();

        void compute();

        void create();

    private:

        void update_ee();

        void update_body();

        void create_body();

        void create_ee();

        glm::vec3 body_color() {
            if(robot_id_ == 0)return {0.8, 0.5, 0.5};
            else if(robot_id_ == 2) return {0.5, 0.8, 0.5};
            return {0.5, 0.5, 0.8};
        }

    };
}


#endif //ROBOTRENDER_H
