//
// Created by Ziqi Wang on 19.02.2024.
//

#include "render/RobotRender.h"
#include "imgui.h"

namespace render {
    RobotRender::RobotRender(std::string name, Eigen::Vector3d base_pos) {
        robot_ = std::make_shared<robot::Robot>(name);
        robot_->baseT_.xyz = base_pos;
        robot_->eeT_.xyz = Eigen::Vector3d(0.2, 0 , 0);

        create();
    }

    RobotRender::~RobotRender() {
        for(auto obj: robot_body_) {
            polyscope::removeStructure(obj);
        }

        for(auto obj: robot_ee_) {
            polyscope::removeStructure(obj);
        }

        polyscope::removeGroup(robot_group);
        polyscope::removeGroup(ee_group);
    }

    void RobotRender::gui()
    {
        std::string headname = robot_->name_ + "_gui";
        if (ImGui::CollapsingHeader(headname.c_str())) {
            for(int id = 0; id < j_.size(); id++)
            {
                float *data = j_.data() + id;
                std::string name = robot_->name_ +  + ": joint angle " + std::to_string(id);
                if(ImGui::SliderFloat(name.c_str(), data, (float) -M_PI,(float) M_PI)) {
                    update();
                }
            }

            for(int id = 0; id < ee_xyz_.size(); id++)
            {
                float *data = ee_xyz_.data() + id;
                std::string name = robot_->name_ +  + ": xyz " + std::to_string(id);
                if(ImGui::SliderFloat(name.c_str(), data, (float) -3,(float) 3)) {
                    compute();
                }
            }

            for(int id = 0; id < ee_rpy_.size(); id++)
            {
                float *data = ee_rpy_.data() + id;
                std::string name = robot_->name_ + ": rpy " + std::to_string(id);
                if(ImGui::SliderFloat(name.c_str(), data, (float) -M_PI,(float) M_PI)) {
                    compute();
                }
            }

            std::string name = robot_->name_ + ": ik sol";
            if(ImGui::SliderInt(name.c_str(), &iksol_,  0,7))
            {
                compute();
            }
        }
    }

    void RobotRender::update(){
        update_body();
        update_ee();
    }

    void RobotRender::compute() {
        util::Transform EE;
        EE.xyz = ee_xyz_.cast<double>();
        EE.rpy = ee_rpy_.cast<double>();
        std::vector<Eigen::VectorXd> solutions = robot_->inverseEE(EE);
        if(!solutions.empty() && iksol_ >= 0 && iksol_ < solutions.size()) {
            j_ = solutions[iksol_].cast<float>();
            update();
        }
    }

    void RobotRender::create() {
        j_.resize(6);
        j_.setZero();

        util::Transform EE = robot_->forwardEE(j_.cast<double>());
        ee_xyz_ = EE.xyz.cast<float>();
        ee_rpy_ = EE.rpy.cast<float>();

        create_body();
        create_ee();

        update();
    }

    void RobotRender::create_body()
    {
        robot_group = polyscope::createGroup(robot_->name_ +"_body");
        for(int id = 0; id < robot_->links_.size(); id++)
        {
            std::string name = robot_->name_ +"_body_" + robot_->links_[id]->name;
            auto obj = polyscope::registerSurfaceMesh(name.c_str(), robot_->links_[id]->visual_meshV, robot_->links_[id]->visual_meshF);
            obj->addToGroup(*robot_group);
            robot_body_.push_back(obj);
            obj->setSurfaceColor(glm::vec3(0.8, 0.8, 0.8));
        }
        robot_group->setHideDescendantsFromStructureLists(true);
        robot_group->setShowChildDetails(false);
    }

    void RobotRender::create_ee() {
        ee_group = polyscope::createGroup(robot_->name_ +"_ee");

        for(int id = 0; id < 3; id++)
        {
            Eigen::MatrixXd V(2, 3); V.setZero();
            Eigen::MatrixXi E(1, 2);
            E.row(0) = Eigen::RowVector2i(0, 1);
            std::string name = robot_->name_ + "_EEframe_" + std::to_string(id);
            auto edge = polyscope::registerCurveNetwork(name.c_str(), V, E);
            edge->setRadius(ee_arrow_radius_, false);
            glm::vec3 color = {0, 0 ,0};
            color[id] = 1;
            edge->setColor(color);
            edge->addToGroup(*ee_group);
            robot_ee_.push_back(edge);
        }

        ee_group->setHideDescendantsFromStructureLists(true);
        ee_group->setShowChildDetails(false);
    }

    void RobotRender::update_ee()
    {
        util::Transform EE = robot_->forwardEE(j_.cast<double>());

        for(int id = 0; id < 3; id++)
        {
            Eigen::MatrixXd V(2, 3);
            Eigen::Vector3d origin = EE.xyz;
            Eigen::Vector3d axis = EE.rot().col(id);
            V.row(0) = origin;
            V.row(1) = origin + axis * ee_arrow_length_;
            std::string name = robot_->name_ + "_EEframe" + std::to_string(id);
            robot_ee_[id]->updateNodePositions(V);
        }
    }

    void RobotRender::update_body()
    {
        std::vector<Eigen::Matrix4d> jointT, linkT;
        robot_->forward(j_.cast<double>(), jointT, linkT);
        for(int id = 0; id < robot_->links_.size(); id++) {
            Eigen::MatrixXd V = robot_->links_[id]->visual_meshV;
            Eigen::MatrixXd Vh(V.rows(), 4); Vh.setOnes();
            Vh.block(0, 0, V.rows(), 3) = V;

            Eigen::MatrixXd VhT = Vh * linkT[id].transpose();
            V = VhT.block(0, 0, V.rows(), 3);
            robot_body_[id]->updateVertexPositions(V);
        }
    }
}
