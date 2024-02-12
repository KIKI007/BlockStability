//
// Created by 汪子琦 on 05.09.22.
//

#ifndef ROBO_CRAFT_READOBJ_H
#define ROBO_CRAFT_READOBJ_H
#include <Eigen/Dense>
#include <vector>

namespace rigid_block {
    class readOBJ {
    public:
        std::vector<Eigen::MatrixXd> Vs_;

        std::vector<Eigen::MatrixXd> Ns_;

        std::vector<Eigen::MatrixXi> Fs_;

        int vCount = 0;

    public:

        void split(std::vector<std::string> &words, const std::string &line, const char &separator);

        void loadFromFile(std::string filename);

        void addObject(const std::vector<Eigen::Vector3d> &vV, const std::vector<Eigen::Vector3i> &vF,
                       const std::vector<Eigen::Vector3d> &vN);

        void addVertex(const std::string &line, std::vector<Eigen::Vector3d> &vV);

        void addFace(const std::string &line, std::vector<Eigen::Vector3i> &vF);

        void addNormal(const std::string &line, std::vector<Eigen::Vector3d> &vN);

        void list_to_matrix(const std::vector<Eigen::Vector3d> &vV, Eigen::MatrixXd &V);

        void list_to_matrix(const std::vector<Eigen::Vector3i> &vf, Eigen::MatrixXi &F);
    };
}


#endif  //ROBO_CRAFT_READOBJ_H
