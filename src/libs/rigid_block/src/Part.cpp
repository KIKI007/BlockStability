//
// Created by 汪子琦 on 04.09.22.
//
#include "rigid_block/Part.h"
#include "iostream"
#include <algorithm>
namespace rigid_block {
    double Part::volume() {
        double volume = 0;

        // Accumulate volume value for each triangle
        for (size_t i = 0; i < F_.rows(); i++)
        {
            Eigen::Vector3d v0, v1, v2;
            v0 = V_.row(F_(i, 0));
            v1 = V_.row(F_(i, 1));
            v2 = V_.row(F_(i, 2));

            Eigen::Vector3d crossVec = -1.0f * (v2 - v0).cross(v1 - v0);
            double dotP = v0.dot(crossVec);
            volume += dotP;
        }

        volume = volume / 6.0;
        return volume;
    }
    Eigen::Vector3d Part::centroid()
    {
        Eigen::Vector3d centroid = Eigen::Vector3d(0, 0, 0);

        // Save the 3 major axes
        Eigen::Vector3d axes[3];
        axes[0] = Eigen::Vector3d(1, 0, 0);
        axes[1] = Eigen::Vector3d(0, 1, 0);
        axes[2] = Eigen::Vector3d(0, 0, 1);

        // Accumulate centroid value for each major axes
        for (int i = 0; i < 3; i++)
        {
            Eigen::Vector3d axis = axes[i];

            for (size_t j = 0; j < F_.rows(); j++)
            {
                Eigen::Vector3d v2, v1, v0;
                v2 = V_.row(F_(j, 2));
                v1 = V_.row(F_(j, 1));
                v0 = V_.row(F_(j, 0));

                Eigen::Vector3d crossVec = -1.0f * (v2 - v0).cross(v1 - v0);

                centroid[i] += (1 / 24.0f) * (crossVec.dot(axis)) *
                               (pow((v0 + v1).dot(axis), 2) +
                                pow((v1 + v2).dot(axis), 2) +
                                pow((v2 + v0).dot(axis), 2));
            }
        }

        // Compute volume and centroid
        double v = volume();

        if(v > 1E-6){
            centroid = centroid / (2.0 * v);
        }
        else{
            centroid = Eigen::Vector3d(0, 0, 0);
        }

        return centroid;
    }

    Eigen::Vector3d Part::normal(int fid) {
        if(fid >= 0 && fid < F_.rows()){
            Eigen::Vector3d v0 = V_.row(F_(fid, 0));
            Eigen::Vector3d v1 = V_.row(F_(fid, 1));
            Eigen::Vector3d v2 = V_.row(F_(fid, 2));
            Eigen::Vector3d normal = ((v1 - v0).cross(v2 - v0));
            return normal.normalized();
        }
        return Eigen::Vector3d(0, 0, 0);
    }

    Eigen::Vector3d Part::center(int fid) {
        if(fid >= 0 && fid < F_.rows()){
            Eigen::Vector3d v0 = V_.row(F_(fid, 0));
            Eigen::Vector3d v1 = V_.row(F_(fid, 1));
            Eigen::Vector3d v2 = V_.row(F_(fid, 2));
            return (v0 + v1 + v2) / 3;
        }
        return Eigen::Vector3d(0, 0, 0);
    }

    std::vector<Eigen::Vector3d> Part::face(int fid){
        if(fid >= 0 && fid < F_.rows()){
            Eigen::Vector3d v0 = V_.row(F_(fid, 0));
            Eigen::Vector3d v1 = V_.row(F_(fid, 1));
            Eigen::Vector3d v2 = V_.row(F_(fid, 2));
            return {v0, v1, v2};
        }
        return {};
    }

    double Part::computeDiagnalLength() {
        Eigen::Vector3d minCoord(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        Eigen::Vector3d maxCoord(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
        for(int ir = 0; ir < V_.rows(); ir++) {
            Eigen::Vector3d pt = V_.row(ir);
            minCoord = minCoord.cwiseMin(pt);
            maxCoord = maxCoord.cwiseMax(pt);
        }
        return (maxCoord - minCoord).norm() / 2;
    }

    std::vector<util::Transform> Part::eeAnchor() {
        std::vector<util::Transform> ts;
        std::vector<double> areas;
        for(int id = 0; id < F_.rows(); id++)
        {
            Eigen::Vector3d p0 = V_.row(F_(id, 0));
            Eigen::Vector3d p1 = V_.row(F_(id, 1));
            Eigen::Vector3d p2 = V_.row(F_(id, 2));
            Eigen::Vector3d ct = (p0 + p1 + p2) / 3;
            Eigen::Vector3d n = -(p1 - p0).cross(p2 - p0); n.normalize();

            Eigen::Vector3d yaxis = Eigen::Vector3d(1, 0, 0).cross(n);
            if(yaxis.norm() < 1E-6) yaxis = Eigen::Vector3d(0, 1, 0).cross(n);
            yaxis.normalize();
            Eigen::Vector3d xaxis = yaxis.cross(n); xaxis.normalize();

            Eigen::Matrix3d rot; rot.setZero();
            rot.col(0) = n;
            rot.col(1) = xaxis;
            rot.col(2) = yaxis;

            util::Transform t;
            t.from_rot(rot);
            t.xyz = ct;
            ts.push_back(t);
            areas.push_back((p1 - p0).cross(p2 - p0).norm());
        }
        auto it = std::max_element(areas.begin(), areas.end());
        util::Transform t = ts[it - areas.begin()];
        ts.clear();
        ts.push_back(t);
        return ts;
    }

}

