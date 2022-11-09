//
// Created by 汪子琦 on 05.09.22.
//

#include "block/ContactGraphForce.h"
#include "iostream"
namespace block_stability{
void ContactGraphForce::addContact3(int partIDA,
                                    int partIDB,
                                    const Eigen::Vector3d& normal,
                                    const std::vector<Eigen::Vector3d>& normal_point)
{
    for(int id = 0; id < normal_point.size(); id++){
        contacts_.push_back(std::make_tuple(partIDA, partIDB, normal, normal_point[id]));
    }
}

void ContactGraphForce::computeForceMatrix(std::vector<Eigen::Triplet<double>>& A, bool tension)
{
    for(int id = 0; id < dynamic_contacts_.size(); id++)
    {
        int partIDA = std::get<0>(dynamic_contacts_[id]);
        int partIDB = std::get<1>(dynamic_contacts_[id]);
        Eigen::Vector3d n = std::get<2>(dynamic_contacts_[id]);
        Eigen::Vector3d r = std::get<3>(dynamic_contacts_[id]);
        Eigen::Vector3d t1, t2;
        computeFrictionDir(n, t1, t2);

        std::vector<int> partIDs = {partIDA, partIDB};

        std::vector<Eigen::Vector3d> drts;
        if(tension) drts = {n, -n, t1, t2};
        else drts = {n, t1, t2};

        for(int pI = 0; pI < 2; pI++)
        {
            int sign = (pI == 0 ? -1 : 1);
            int partID = partIDs[pI];

            if(partID == -1) continue ;

            //force
            for(int jd = 0; jd < 3; jd++)
            {
                int row = partID * 6 + jd;
                for(int kd = 0; kd < drts.size(); kd++)
                {
                    int col = drts.size() * id + kd;
                    Eigen::Vector3d f = drts[kd];
                    A.push_back(Eigen::Triplet(row, col, f[jd] * sign));
                }
            }

            //torque
            for(int jd = 0; jd < 3; jd++)
            {
                int row = partID * 6 + jd + 3;
                for(int kd = 0; kd < drts.size(); kd++)
                {
                    Eigen::Vector3d m = r.cross(drts[kd]);
                    int col = drts.size() * id + kd;
                    A.push_back(Eigen::Triplet(row, col, m[jd] * sign));
                }
            }
        }
    }
}

void ContactGraphForce::computeGravity(Eigen::VectorXd& w)
{
    w = Eigen::VectorXd::Zero(n_dynamic_part_ * 6);
    for(int id = 0; id < n_part_; id++)
    {
        if(mapOldToNew_[id] == -1)
            continue;

        int partID = mapOldToNew_[id];

        Eigen::Vector3d force(0, -mass_[id], 0);
        Eigen::Vector3d r = centroid_[partID];
        Eigen::Vector3d torque = r.cross(force);
        w.segment(partID * 6, 3) = force;
        w.segment(partID * 6 + 3, 3) = torque;
    }
    return;
}

void ContactGraphForce::setFixedParts(const std::vector<int>& fixedParts)
{
    mapOldToNew_.clear();
    mapNewToOld_.clear();
    computeMap(fixedParts);

    for(int id = 0; id < contacts_.size(); id++) {

        int partIDA = std::get<0>(contacts_[id]);
        int partIDB = std::get<1>(contacts_[id]);

        //both parts are fixed
        auto findA = mapOldToNew_.find(partIDA);
        auto findB = mapOldToNew_.find(partIDB);
        if(findA->second == -1 && findB->second == -1){
            //useless contact
        }
        else{
            int newA = findA->second;
            int newB = findB->second;
            Eigen::Vector3d normal = std::get<2>(contacts_[id]);
            Eigen::Vector3d point = std::get<3>(contacts_[id]);
            dynamic_contacts_.push_back({newA, newB, normal, point});
        }
    }
}

void ContactGraphForce::computeMap(const std::vector<int>& fixedParts) {
    std::vector<bool> fixed;
    fixed.resize(n_part_, false);
    for(int id = 0; id < fixedParts.size(); id++){
        fixed[fixedParts[id]] = true;
    }

    n_dynamic_part_ = 0;
    for(int id = 0; id < n_part(); id++){
        if(fixed[id] == false)
        {
            mapNewToOld_[n_dynamic_part_] = id;
            mapOldToNew_[id] = n_dynamic_part_;
            n_dynamic_part_++;
        }
        else{
            mapOldToNew_[id] = -1;
        }
    }
}

void ContactGraphForce::computeFrictionDir(const Eigen::Vector3d& n, Eigen::Vector3d& t1, Eigen::Vector3d& t2) {
    t1 = n.cross(Eigen::Vector3d(1, 0, 0));
    if(t1.norm() < 1E-4){
        t1 =  n.cross(Eigen::Vector3d(0, 1, 0));
    }
    t1.normalize();
    t2 = n.cross(t1);
    t2.normalize();
}


}

