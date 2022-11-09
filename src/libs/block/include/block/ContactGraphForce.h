//
// Created by 汪子琦 on 05.09.22.
//

#ifndef ROBO_CRAFT_CONTACTGRAPHFORCE_H
#define ROBO_CRAFT_CONTACTGRAPHFORCE_H
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include <map>
namespace block_stability
{
    class ContactGraphForce
    {
    public:

        typedef Eigen::SparseMatrix<double> SparseMat;

    public:

        std::vector<std::tuple<int, int, Eigen::Vector3d, Eigen::Vector3d>> contacts_;

        std::vector<double> mass_;

        std::vector<Eigen::Vector3d> centroid_;

        int n_part_;

        double density_ = 1;

    public:

        std::vector<std::tuple<int, int, Eigen::Vector3d, Eigen::Vector3d>> dynamic_contacts_;

        int n_dynamic_part_;

        std::map<int, int> mapOldToNew_; //partID from old to new

        std::map<int, int> mapNewToOld_; //partID from new to old

    public:

        ContactGraphForce(int nParts): n_part_(nParts){
            mass_.resize(nParts, 0.0);
            centroid_.resize(nParts, Eigen::Vector3d::Zero());
        }

        ContactGraphForce(const ContactGraphForce &graph){
            n_part_ = graph.n_part_;
            mass_ = graph.mass_;
            centroid_ = graph.centroid_;
            contacts_ = graph.contacts_;

            dynamic_contacts_ = graph.dynamic_contacts_;
            n_dynamic_part_ = graph.n_dynamic_part_;
            mapOldToNew_ = graph.mapOldToNew_;
            mapNewToOld_ = graph.mapNewToOld_;
        }

    public:

        void addContact3(int partIDA, int partIDB, const Eigen::Vector3d &normal, const std::vector<Eigen::Vector3d> &normal_point);

        void setPart3(int partID, double area, Eigen::Vector3d ct)
        {
            mass_[partID] = area * density_ ;
            centroid_[partID] = ct;
        }

        int n_contact(){return contacts_.size();}

        int n_part(){return n_part_;}

        int n_dynamic_part(){return n_dynamic_part_;}

        int n_dynamic_contact(){return dynamic_contacts_.size();}

        double density(){return density_;}

        double mass(int index){return mass_.at(index);}

        Eigen::Vector3d centroid(int index){return centroid_.at(index);}

        std::tuple<int, int, Eigen::Vector3d, Eigen::Vector3d> contact(int index){return contacts_.at(index);}

    public:

        //Bin
        void computeForceMatrix(std::vector<Eigen::Triplet<double>> &A, bool tension = true);

        void computeGravity(Eigen::VectorXd &w);

        void computeFrictionDir(const Eigen::Vector3d &n, Eigen::Vector3d &t1, Eigen::Vector3d &t2);

        void setFixedParts(const std::vector<int> &fixedParts);

        void computeMap(const std::vector<int> &fixedParts);
    };
}


#endif  //ROBO_CRAFT_CONTACTGRAPHFORCE_H
