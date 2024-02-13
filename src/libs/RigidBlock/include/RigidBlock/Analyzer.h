//
// Created by 汪子琦 on 05.09.22.
//

#ifndef ROBO_CRAFT_CONTACTGRAPHFORCE_H
#define ROBO_CRAFT_CONTACTGRAPHFORCE_H
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include <map>
#include "gurobi_c++.h"

using std::vector;

namespace rigid_block
{

    class Arrow
    {
    public:
        Eigen::MatrixXd V;
        Eigen::MatrixXi E;
        void from_points(const std::vector<Eigen::Vector3d> &points);
        std::string name;
        Eigen::Vector3d color;
    };

    class ContactPoint{
    public:
        int partIDA;
        int partIDB;
        Eigen::Vector3d contact_point;
        Eigen::Vector3d contact_normal;
        Eigen::Vector3d contact_friction_t1;
        Eigen::Vector3d contact_friction_t2;
    };

    class AnalysisResult
    {
    public:

        Eigen::VectorXd internal_contact_forces;
        Eigen::VectorXd support_forces;
        std::vector<ContactPoint> contact_points;
        Eigen::VectorXd gravity_forces;
        std::vector<Eigen::Vector3d> centroid;
        bool with_tension;

        std::vector<Arrow> computeArrows(int partID);
    };

    class Analyzer
    {
    public:
        enum PartStatus {
            Uninstalled = 0,
            Installed = 1,
            Fixed = 2
        };
    private:

        // contact list
        // partIDA, partIDB, normal, contact point
        // id is -1 if the corresponding block is fixed
        std::vector<ContactPoint> contact_points_;

        //blocks' weight
        std::vector<double> mass_;

        //blocks' centroid
        std::vector<Eigen::Vector3d> centroid_;

        //blocks' status
        //0 uninstalled
        //1 installed
        //2 fixed/grounded
        std::vector<PartStatus> status_;

        // number of part
        int n_part_;

        double friction_mu_;

    private:

        std::vector<Eigen::Triplet<double>> equlibrium_mat_triplets_;

        Eigen::VectorXd gravity_;

    public:

        Analyzer(int nParts): n_part_(nParts)
        {
            mass_.resize(nParts, 0.0);
            centroid_.resize(nParts, Eigen::Vector3d::Zero());
            status_.resize(nParts, Uninstalled);
        }

        Analyzer(const Analyzer &analyzer)
        {
            n_part_ = analyzer.n_part_;
            mass_ = analyzer.mass_;
            centroid_ = analyzer.centroid_;
            contact_points_ = analyzer.contact_points_;
        }

    public:

        int n_contact(){return contact_points_.size();}

        int n_part(){return n_part_;}

        double mass(int index){return mass_.at(index);}

        Eigen::Vector3d centroid(int index){return centroid_.at(index);}

        ContactPoint contact(int index){return contact_points_.at(index);}


    public: //check stability
        void updateFrictionCeoff(double mu){friction_mu_ = mu;}

        void addContact(int partIDA, int partIDB, const Eigen::Vector3d &normal, const std::vector<Eigen::Vector3d> &normal_point);

        void updatePart(int partID, double mass, Eigen::Vector3d ct);

        void updatePartStatus(int partID, PartStatus status) {
            if(partID >= 0 && partID < n_part()) {
                status_[partID] = status;
            }
        }

        void updateEquilibriumMatrix(bool tension = true);

        void updateGravity();

        bool solve(AnalysisResult &result, bool tension = true);

        void getInternalForcesArrow(AnalysisResult &result);

    private: //setup the gurobi solver

        GRBVar *createContactForceVars(GRBModel &model, bool tension = true);

        GRBVar *createSupportForceVars(GRBModel &model);

        void setForceEquilibrium(GRBModel &model, GRBVar *contactForce, GRBVar *supportForce);

        void setFrictionCone(GRBModel &model, GRBVar *force, bool tension = true);

        void computeFrictionDir(const Eigen::Vector3d &n, Eigen::Vector3d &t1, Eigen::Vector3d &t2);
    };
}


#endif  //ROBO_CRAFT_CONTACTGRAPHFORCE_H
