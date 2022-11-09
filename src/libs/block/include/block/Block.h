//
// Created by 汪子琦 on 04.09.22.
//

#ifndef ROBO_CRAFT_BLOCK_H
#define ROBO_CRAFT_BLOCK_H
#include "Eigen/Dense"
#include <vector>
namespace block
{

class Block
{
public:

    Eigen::MatrixXd V_;

    Eigen::MatrixXi F_;

    Eigen::MatrixXd N_;

    int partID_;

    bool ground_ = false;

    double rho = 1;

public:

    double volume();

    Eigen::Vector3d centroid();

    Eigen::Vector3d normal(int fid);

    Eigen::Vector3d center(int fid);

    std::vector<Eigen::Vector3d> face(int fid);


};

}


#endif  //ROBO_CRAFT_BLOCK_H
