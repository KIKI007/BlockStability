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

namespace render {
    class RobotRender {
    public:

        bool show_robot_ = true;

        bool show_ee_ = false;

    public:

        Eigen::VectorXf j_;

        Eigen::Vector3f ee_xyz_;

        Eigen::Vector3f ee_rpy_;

        int iksol_ = 2;

        std::shared_ptr<robot::Robot> robot_;

    public:

        std::vector<polyscope::SurfaceMesh *> robot_body_;

        std::vector<polyscope::CurveNetwork *> robot_ee_;

        polyscope::Group *robot_group, *ee_group;


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

    };
}


#endif //ROBOTRENDER_H
