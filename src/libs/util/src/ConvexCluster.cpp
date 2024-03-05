//
// Created by Ziqi Wang on 04.03.2024.
//
#include "util/ConvexCluster.h"

std::vector<int> ConvexCluster::computeClusterByPlane(const std::vector<Eigen::Vector3d> &points,
                                                      const std::vector<Eigen::Vector3d> &normals) {
    std::vector<int> groupIDs;
    groupIDs.resize(points.size(), -1);
    int count = 0;
    for (int id = 0; id < points.size(); id++) {
        if (groupIDs[id] == -1)
        {
            Eigen::Vector3d pi = points[id];
            Eigen::Vector3d ni = normals[id];
            groupIDs[id] = count++;
            for (int jd = id + 1; jd < points.size(); jd++) {
                if (groupIDs[jd] == -1) {
                    Eigen::Vector3d pj = points[jd];
                    Eigen::Vector3d nj = normals[jd];
                    if ((ni - nj).norm() < error_small_normal_
                        && abs((pi - pj).dot(ni)) < error_small_distance_) {
                        groupIDs[jd] = groupIDs[id];
                    }
                }
            }
        }
    }
    return groupIDs;
}

void ConvexCluster::computeConvexHull(

const std::vector<Eigen::Vector3d> &points,
    const std::vector<Eigen::Vector3d> &normals,
    std::vector<std::vector<Eigen::Vector3d>> &out_points,
    std::vector<Eigen::Vector3d> &out_normal) {
    std::vector<int> groupIDs = computeClusterByPlane(points, normals);
    int select_group_id = 0;

    ConvexHull2D<double> convexhull;
    out_normal.clear();
    out_points.clear();

    while (true) {
        std::vector<Eigen::Vector3d> hull_pts;
        Eigen::Vector3d hull_n;
        for (int id = 0; id < points.size(); id++) {
            if (groupIDs[id] == select_group_id) {
                hull_pts.push_back(points[id]);
                hull_n = normals[id];
            }
        }
        if (hull_pts.empty()) break;

        //convex hull
        std::vector<Eigen::Vector3d> hull;
        convexhull.compute(hull_pts, hull_n, hull);
        out_points.push_back(simplify(hull));
        out_normal.push_back(hull_n);

        //
        select_group_id ++;
    }

}

std::vector<Eigen::Vector3d> ConvexCluster::simplify(const std::vector<Eigen::Vector3d> &points) {
    std::vector<Eigen::Vector3d> new_points;
    if (!points.empty()) {
        new_points.push_back(points.front());
        for (int id = 1; id < points.size(); id++){
            Eigen::Vector3d pt = points[id];

            //if new_points has only one
            if (new_points.size() == 1) {
                if ((new_points[0] - pt).norm() > error_small_distance_) {
                    new_points.push_back(pt);
                }
            }
            //if new_points has two points
            else if (new_points.size() == 2){
                Eigen::Vector3d p0 = new_points[0];
                Eigen::Vector3d p1 = new_points[1];
                Eigen::Vector3d p01 = p1 - p0;
                Eigen::Vector3d p1t = pt - p1;
                if (p01.cross(p1t).norm() > error_small_normal_) {
                    new_points.push_back(pt);
                } else {
                    new_points.back() = pt;
                }
            } else {
                Eigen::Vector3d p0 = new_points[0];
                Eigen::Vector3d p1 = new_points[1];
                Eigen::Vector3d p2 = new_points[new_points.size() - 2];
                Eigen::Vector3d p3 = new_points[new_points.size() - 1];
                Eigen::Vector3d p23 = p3 - p2;
                Eigen::Vector3d p3t = pt - p3;
                Eigen::Vector3d pt0 = p0 - pt;
                Eigen::Vector3d p01 = p1 - p0;

                if (p3t.cross(pt0).norm() < error_small_normal_) {
                    continue ;
                } else {
                    if (p23.cross(p3t).norm() < error_small_normal_) {
                        new_points.back() = pt;
                        continue ;
                    }

                    if (pt0.cross(p01).norm() < error_small_normal_) {
                        new_points.front() = pt;
                        continue ;
                    }
                }
                new_points.push_back(pt);
            }
        }
    }
    return new_points;
}
