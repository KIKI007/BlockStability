//
// Created by Ziqi Wang on 04.03.2024.
//

#ifndef CONVEXCLUSTER_H
#define CONVEXCLUSTER_H
#include <Eigen/Dense>
#include <vector>
#include "ConvexHull2D.h"
class ConvexCluster{
public:

    double error_small_normal_;
    double error_small_distance_;
public:

    ConvexCluster() {
        error_small_normal_ = 1E-4;
        error_small_distance_ = 1E-2;
    }

    std::vector<int> computeClusterByPlane(const std::vector<Eigen::Vector3d> &points,
                                           const std::vector<Eigen::Vector3d> &normals);

    void  computeConvexHull(const std::vector<Eigen::Vector3d> &points,
                            const std::vector<Eigen::Vector3d> &normals,
                            std::vector<std::vector<Eigen::Vector3d>> &out_points,
                            std::vector<Eigen::Vector3d> &out_normal);

    std::vector<Eigen::Vector3d> simplify(const std::vector<Eigen::Vector3d> &points);

};
#endif //CONVEXCLUSTER_H
