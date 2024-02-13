//
// Created by 汪子琦 on 04.09.22.
//

#include "RigidBlock/Assembly.h"
#include "RigidBlock/util/PolyPolyBoolean.h"
#include "RigidBlock/util/ConvexHull2D.h"

namespace rigid_block {
    std::vector<ContactFace> Assembly::computeContacts(const std::vector<int> &subPartIDs) {
        std::vector<ContactFace> contacts;
        for (int id = 0; id < subPartIDs.size(); id++) {
            for (int jd = id + 1; jd < subPartIDs.size(); jd++) {
                int partIDA = subPartIDs[id];
                int partIDB = subPartIDs[jd];
                std::vector<ContactFace> list_contacts = computeContacts(blocks_[partIDA], blocks_[partIDB]);
                for (int kd = 0; kd < list_contacts.size(); kd++) {
                    list_contacts[kd].partIDA = id;
                    list_contacts[kd].partIDB = jd;
                }
                contacts.insert(contacts.end(), list_contacts.begin(), list_contacts.end());
            }
        }
        return contacts;
    }

    std::vector<ContactFace> Assembly::computeContacts(std::shared_ptr<Part> block1,
                                                   std::shared_ptr<Part> block2) {
        PolyPolyBoolean boolean;

        std::vector<Eigen::Vector3d> points;
        std::vector<Eigen::Vector3d> normals;

        for (int f1_i = 0; f1_i < block1->F_.rows(); f1_i++) {
            Eigen::Vector3d n1_i = block1->normal(f1_i);
            Eigen::Vector3d p1_i = block1->center(f1_i);
            for (int f2_j = 0; f2_j < block2->F_.rows(); f2_j++) {
                Eigen::Vector3d n2_j = block2->normal(f2_j);
                Eigen::Vector3d p2_j = block2->center(f2_j);

                //if two faces have the opposite normal
                //and two faces on the same plane
                //compute the intersection area, or the contact area
                if ((n1_i + n2_j).norm() <= error_small_normal_
                    && abs((p2_j - p1_i).dot(n1_i)) <= error_small_distance_) {
                    PolyPolyBoolean::PolyVector3 poly1_i = block1->face(f1_i);
                    PolyPolyBoolean::PolyVector3 poly2_j = block2->face(f2_j);
                    PolyPolyBoolean::PolysVector3 outs;
                    boolean.computePolygonsIntersection(poly1_i, poly2_j, outs);

                    for (PolyPolyBoolean::PolyVector3 out: outs)
                    {
                        for (int kd = 0; kd < out.size(); kd++)
                        {
                            points.push_back(out[kd]);
                            normals.push_back(n1_i);
                        }
                    }
                }
            }
        }
        return simplifyContact(points, normals);
    }

    void Assembly::toMesh(const std::vector<ContactFace> &contacts, Eigen::MatrixXd &V, Eigen::MatrixXi &F)
    {
        int nV = 0;
        int nF = 0;
        for (int id = 0; id < contacts.size(); id++) {
            auto contact = contacts[id];
            nV += contact.points.size();
            nF += contact.points.size() - 2;
        }

        V = Eigen::MatrixXd(nV, 3);
        F = Eigen::MatrixXi(nF, 3);

        int iV = 0;
        int iF = 0;

        for (int id = 0; id < contacts.size(); id++) {
            auto contact = contacts[id];
            int nV = contact.points.size();
            for (int jd = 0; jd < nV; jd++) {
                V.row(jd + iV) = contact.points[jd];
            }
            for (int jd = 2; jd < nV; jd++) {
                F(jd - 2 + iF, 0) = iV;
                F(jd - 2 + iF, 1) = iV + jd - 1;
                F(jd - 2 + iF, 2) = iV + jd;
            }
            iV += nV;
            iF += nV - 2;
        }
    }

    std::vector<ContactFace> Assembly::simplifyContact(const std::vector<Eigen::Vector3d> &points,
                                                       const std::vector<Eigen::Vector3d> &normals)
    {
        std::vector<ContactFace> contacts;

        std::vector<int> groupIDs;
        groupIDs.resize(points.size(), -1);
        int count = 0;
        for (int id = 0; id < points.size(); id++) {
            if (groupIDs[id] == -1) {
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

        ConvexHull2D<double> convexhull;
        for (int id = 0; id < count; id++) {
            std::vector<Eigen::Vector3d> hull_pts;
            Eigen::Vector3d hull_n;
            for (int jd = 0; jd < points.size(); jd++) {
                if (groupIDs[jd] == id) {
                    hull_pts.push_back(points[jd]);
                    hull_n = normals[jd];
                }
            }
            std::vector<Eigen::Vector3d> hull;
            convexhull.compute(hull_pts, hull_n, hull);
            ContactFace newContact;
            if (!hull.empty())
            {
                std::vector<Eigen::Vector3d> results;
                results.push_back(hull.front());
                for (int jd = 1; jd < hull.size(); jd++) {
                    Eigen::Vector3d pt = hull[jd];
                    if (results.size() == 1) {
                        if ((results[0] - pt).norm() > error_small_distance_) {
                            results.push_back(pt);
                        }
                    } else if (results.size() == 2) {
                        Eigen::Vector3d p0 = results[0];
                        Eigen::Vector3d p1 = results[1];
                        Eigen::Vector3d p01 = p1 - p0;
                        Eigen::Vector3d p1t = pt - p1;
                        if (p01.cross(p1t).norm() > error_small_normal_) {
                            results.push_back(pt);
                        } else {
                            results[1] = pt;
                        }
                    } else {
                        Eigen::Vector3d p0 = results[0];
                        Eigen::Vector3d p1 = results[1];
                        Eigen::Vector3d p2 = results[results.size() - 2];
                        Eigen::Vector3d p3 = results[results.size() - 1];
                        Eigen::Vector3d p23 = p3 - p2;
                        Eigen::Vector3d p3t = pt - p3;
                        Eigen::Vector3d pt0 = p0 - pt;
                        Eigen::Vector3d p01 = p1 - p0;

                        if (p3t.cross(pt0).norm() < error_small_normal_) {
                            continue ;
                        } else {
                            if (p23.cross(p3t).norm() < error_small_normal_) {
                                results.back() = pt;
                                continue ;
                            }

                            if (pt0.cross(p01).norm() < error_small_normal_) {
                                results.front() = pt;
                                continue ;
                            }
                        }
                        results.push_back(pt);
                    }
                }
                newContact.points = results;
                newContact.normal = hull_n;
                contacts.push_back(newContact);
            }
        }
        return contacts;
    }

    void Assembly::loadFromFile(std::string filename) {
        readOBJ readObj;
        readObj.loadFromFile(filename);
        for (int id = 0; id < readObj.Vs_.size(); id++) {
            std::shared_ptr<Part> block = std::make_shared<Part>();
            block->V_ = readObj.Vs_[id];
            block->F_ = readObj.Fs_[id];
            block->N_ = readObj.Ns_[id];
            block->partID_ = id;
            block->ground_ = false;
            blocks_.push_back(block);
        }
        addGroundPlane();
        updateGroundBlocks();
    }

    void Assembly::updateGroundBlocks() {
        for (int id = 0; id < blocks_.size(); id++) {
            std::shared_ptr<Part> block = blocks_[id];
            std::vector<ContactFace> contacts;
            contacts = computeContacts(block, ground_plane_);
            if (!contacts.empty()) {
                block->ground_ = true;
            }
        }
    }

    void Assembly::addGroundPlane() {
        Eigen::Vector3d minCoord, maxCoord;
        for (int id = 0; id < blocks_.size(); id++) {
            for (int kd = 0; kd < 3; kd++) {
                Eigen::VectorXd col = blocks_[id]->V_.col(kd);

                if (id == 0) minCoord[kd] = col.minCoeff();
                else minCoord[kd] = std::min(col.minCoeff(), minCoord[kd]);

                if (id == 0) maxCoord[kd] = col.maxCoeff();
                else maxCoord[kd] = std::max(col.maxCoeff(), maxCoord[kd]);
            }
        }

        double height = minCoord[1];

        Eigen::Vector3d center = (minCoord + maxCoord) / 2;
        Eigen::Vector3d size = (maxCoord - minCoord) / 2;
        size *= 1.5; //scale the plane 1.5x to cover the structure's base
        minCoord = center - size;
        maxCoord = center + size;

        ground_plane_ = std::make_shared<Part>();
        ground_plane_->V_ = Eigen::MatrixXd(4, 3);
        ground_plane_->V_ << minCoord[0], height, minCoord[2],
                minCoord[0], height, maxCoord[2],
                maxCoord[0], height, maxCoord[2],
                maxCoord[0], height, minCoord[2];

        ground_plane_->F_ = Eigen::MatrixXi(2, 3);
        ground_plane_->F_ << 0, 1, 2,
                0, 2, 3;
        ground_plane_->N_ = Eigen::MatrixXd(4, 3);
        ground_plane_->N_ << 0, 1, 0,
                0, 1, 0,
                0, 1, 0,
                0, 1, 0;

        ground_plane_->partID_ = -1;
        ground_plane_->ground_ = true;
    }

    std::shared_ptr<Analyzer> Assembly::createAnalyzer()
    {
        std::shared_ptr<Analyzer> analyzer
        = std::make_shared<Analyzer>(blocks_.size());
        analyzer->updateFrictionCeoff(friction_mu_);

        std::vector<int> partIDs;
        double length = computeAvgDiagnalLength();

        for (int ipart = 0; ipart < blocks_.size(); ipart++) {
            partIDs.push_back(ipart);
            analyzer->updatePart(ipart, blocks_[ipart]->volume() / pow(length, 3.0), blocks_[ipart]->centroid());
            if (blocks_[ipart]->ground_) {
                analyzer->updatePartStatus(ipart, Analyzer::Fixed);
            }
            else {
                analyzer->updatePartStatus(ipart, Analyzer::Installed);
            }
        }

        std::vector<ContactFace> contacts;
        contacts = computeContacts(partIDs);

        for (int id = 0; id < contacts.size(); id++) {
            analyzer->addContact(contacts[id].partIDA, contacts[id].partIDB, contacts[id].normal, contacts[id].points);
        }

        analyzer->updateEquilibriumMatrix();
        analyzer->updateGravity();

        return analyzer;
    }

    double Assembly::computeAvgDiagnalLength()
    {
        double length = 0;
        for(int partID = 0; partID < blocks_.size(); partID ++) {
            length += blocks_[partID]->computeDiagnalLength();
        }
        length /= blocks_.size();
        return length;
    }
}
