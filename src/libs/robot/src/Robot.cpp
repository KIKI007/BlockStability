//
// Created by Ziqi Wang on 15.02.2024.
//

#include "robot/Robot.h"
#include <iostream>
#include "util/readOBJ.h"
#include "robot/ikfast.h"
#include <memory>
#include "QuickHull.hpp"
#include "ccd/ccd.h"


robot::Robot::Robot(std::string name) {
    name_ = name;
    tinyxml2::XMLDocument doc;
    std::string robot_str = ROBOCRAFT_DATA_FOLDER "/robots/irb4600/irb4600.urdf";
    std::string robot_folder = ROBOCRAFT_DATA_FOLDER "/robots/irb4600";
    doc.LoadFile(robot_str.c_str());
    load(robot_folder, doc);
}

void robot::Robot::forward(const Eigen::VectorXd &joint_angle, std::vector<Eigen::Matrix4d> &jointT, std::vector<Eigen::Matrix4d> &linkT)
{
    Eigen::Matrix4d T = baseT_.mat();
    std::shared_ptr<RobotLink> link = root_.lock();
    while(link)
    {
        Eigen::MatrixXd Ti = T * link->transf.mat();
        linkT.push_back(Ti);

        std::shared_ptr<RobotJoint> joint = link->child_joint.lock();
        if(joint == nullptr) break;

        T *= joint->transf.mat();
        double angle = joint_angle[joint->state_id];
        Eigen::Matrix3d R3 = Eigen::AngleAxis<double>(angle, joint->axis_xyz).toRotationMatrix();
        Eigen::Matrix4d R4; R4.setIdentity();
        R4.block(0, 0, 3, 3) = R3;
        T *= R4;
        link = joint->child_link.lock();
        jointT.push_back(T);
    }
}

void robot::Robot::load(const std::string &folder_name, tinyxml2::XMLDocument &robot_urdf)
{
    tinyxml2::XMLElement* node =  robot_urdf.FirstChildElement("robot");
    for (tinyxml2::XMLElement* child = node->FirstChildElement(); child != NULL; child = child->NextSiblingElement()){
        std::string tag = child->Value();
        if(tag == "link")
        {
            std::string name = child->Attribute("name");
            printf("loading %s ...\n", name.c_str());

            tinyxml2::XMLElement* visual_node = child->FirstChildElement("visual");
            //tinyxml2::XMLElement* collision_node = child->FirstChildElement("collision");

            tinyxml2::XMLElement* geometry_node = visual_node->FirstChildElement("geometry");
            std::string mesh_name = geometry_node->FirstChildElement("mesh")->Attribute("filename");
            std::string scale_text = geometry_node->FirstChildElement("mesh")->Attribute("scale");
            double scale = std::atof(scale_text.c_str());

            std::string filename = folder_name + "/" + mesh_name;
            util::readOBJ reader;
            reader.loadFromFile(filename);

            std::shared_ptr<RobotLink> link = std::make_shared<RobotLink>();

            // reader.Vs_.push_back(Eigen::MatrixXd(3, 3));
            // reader.Vs_[0] << 0, 0, 1E6,
            // 0, 0, 1E6,
            // 0, 0, 1E6;
            // reader.Fs_.push_back(Eigen::RowVector3i(0, 1, 2));

            if(!reader.Vs_.empty())
            {
                link->visual_meshV = reader.Vs_.front() * scale;
                link->visual_meshF = reader.Fs_.front();

                Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> V = link->visual_meshV;
                quickhull::QuickHull<double> qh;
                auto hull = qh.getConvexHull(V.data(), V.rows(), true, false);
                auto indexBuffer = hull.getIndexBuffer();
                auto vertexBuffer = hull.getVertexBuffer();

                link->collision_meshV = Eigen::MatrixXd(vertexBuffer.size(), 3); link->collision_meshV.setZero();
                link->collision_meshF = Eigen::MatrixXi(indexBuffer.size() / 3, 3); link->collision_meshF.setZero();
                for(int vi = 0; vi < vertexBuffer.size(); vi++)
                {
                    auto pt = vertexBuffer[vi];
                    link->collision_meshV.row(vi) = Eigen::RowVector3d(pt.x, pt.y, pt.z);
                }
                for(int fi = 0; fi < indexBuffer.size() / 3; fi++)
                {
                    link->collision_meshF.row(fi) = Eigen::RowVector3i(indexBuffer[3 * fi], indexBuffer[3 * fi + 1], indexBuffer[3 * fi + 2]);
                }
            }

            link->name = name;
            link->transf = util::Transform (visual_node->FirstChildElement("origin"));
            links_.push_back(link);
        }
        else if(tag == "joint")
        {
            std::shared_ptr<RobotJoint> joint = std::make_shared<RobotJoint>();

            std::string name = child->Attribute("name");
            joint->state_id = joints_.size();
            joint->name = name;
            std::string link_parent_text = child->FirstChildElement("parent")->Attribute("link");
            std::string link_child_text = child->FirstChildElement("child")->Attribute("link");
            joint->parent_link = findLink(link_parent_text);
            joint->child_link = findLink(link_child_text);

            joint->parent_link.lock()->child_joint = joint;
            joint->child_link.lock()->parent_joint = joint;

            joint->transf = util::Transform(child->FirstChildElement("origin"));
            joint->axis_xyz = util::toVec(child->FirstChildElement("axis")->Attribute("xyz"));

            joints_.push_back(joint);
        }
    }

    for(auto &link: links_) {
        if(link->parent_joint.lock() == nullptr) root_ = link;
        if(link->child_joint.lock() == nullptr) EF_ = link;
    }
}

std::shared_ptr<robot::RobotLink> robot::Robot::findLink(std::string name) {
    for(auto link: links_) {
        if(link->name == name)
            return link;
    }
    return nullptr;
}

util::Transform robot::Robot::forwardEE(const Eigen::VectorXd &j)
{
    Eigen::Vector3d EEpos; EEpos.setZero();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> EErot; EErot.setZero();
    ComputeFk(j.data(), EEpos.data(), EErot.data());

    Eigen::Matrix4d EE; EE.setIdentity();
    EE.block(0, 3, 3, 1) = EEpos;
    EE.block(0, 0, 3, 3) = EErot;

    //multiply base util::Transform
    EE = baseT_.mat() * EE * eeT_.mat();

    util::Transform transf;
    transf.xyz = EE.block(0, 3, 3, 1);
    transf.from_rot(EE.block(0, 0, 3, 3));

    return transf;
}

std::vector<Eigen::VectorXd> robot::Robot::inverseEE(const util::Transform &transf)
{
    ikfast::IkSolutionList<double> solutions;
    Eigen::Matrix4d EE = baseT_.mat().inverse() * transf.mat() * eeT_.mat().inverse();

    Eigen::Vector3d EEpos = EE.block(0, 3, 3, 1);
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> EErot = EE.block(0, 0, 3, 3);

    Eigen::VectorXd freeVar(6); freeVar.setZero();

    ComputeIk(EEpos.data(), EErot.data(), freeVar.data(), solutions);

    std::vector<Eigen::VectorXd> results;
    for(int id = 0; id < solutions.GetNumSolutions(); id++) {
        Eigen::VectorXd angle, free;
        angle.resize(6); angle.setZero();
        solutions.GetSolution(id).GetSolution(angle.data(), freeVar.data());
        results.push_back(angle);
    }

    return results;
}

std::vector<Eigen::MatrixXd> robot::Robot::computeGeometry(const Eigen::VectorXd &j, bool visual)
{
    std::vector<Eigen::MatrixXd> Vs;
    std::vector<Eigen::Matrix4d> jointT, linkT;
    forward(j, jointT, linkT);
    for(int id = 0; id < links_.size(); id++)
    {
        Eigen::MatrixXd V;
        if(visual) V = links_[id]->visual_meshV;
        else V = links_[id]->collision_meshV;

        Eigen::MatrixXd Vh(V.rows(), 4); Vh.setOnes();
        Vh.block(0, 0, V.rows(), 3) = V;

        Eigen::MatrixXd VhT = Vh * linkT[id].transpose();
        V = VhT.block(0, 0, V.rows(), 3);
        Vs.push_back(V);
    }
    return Vs;
}

