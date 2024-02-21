//
// Created by Ziqi Wang on 20.02.2024.
//

#ifndef TRANSFORM_H
#define TRANSFORM_H
#include "Eigen/Dense"
#include "tinyxml2.h"
#include "readOBJ.h"

namespace util {
    struct Transform {
        Eigen::Vector3d rpy = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d xyz = Eigen::Vector3d(0, 0, 0);

        Eigen::Matrix4d mat() const{
            Eigen::Matrix4d T;
            T.setIdentity();
            T.block(0, 0, 3, 3) = rot();
            T.block(0, 3, 3, 1) = xyz;
            return T;
        }

        Eigen::Matrix3d rot() const{
            auto q = Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
                     * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ());
            return q.toRotationMatrix();
        }

        void from_rot(const Eigen::Matrix3d &rot){
            rpy = rot.eulerAngles(0, 1, 2);
        }

        Transform() {
        }

        Transform(tinyxml2::XMLElement *node){
            if (node) {
                std::string rpy_text = node->Attribute("rpy");
                std::string xyz_text = node->Attribute("xyz");
                std::vector<std::string> rpy_data, xyz_data;
                rpy = util::toVec(rpy_text);
                xyz = util::toVec(xyz_text);
            }
        }
    };
}


#endif //TRANSFORM_H
