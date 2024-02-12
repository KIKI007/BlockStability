//
// Created by 汪子琦 on 05.09.22.
//

#include "RigidBlock/Analyzer.h"

#include "iostream"

namespace block_stability
{
    void Analyzer::addContact(int partIDA,
                               int partIDB,
                               const Eigen::Vector3d &normal,
                               const std::vector<Eigen::Vector3d> &normal_point) {
        for (int id = 0; id < normal_point.size(); id++) {
            contacts_.push_back(std::make_tuple(partIDA, partIDB, normal, normal_point[id]));
        }
    }

    void Analyzer::updatePart(int partID, double mass, Eigen::Vector3d ct) {
        if (0 <= partID && partID < n_part()) {
            mass_[partID] = mass;
            centroid_[partID] = ct;
        }
    }

    void Analyzer::updateEquilibriumMatrix(bool tension)
    {
        for (int ic = 0; ic < contacts_.size(); ic++)
        {
            int partIDA = std::get<0>(contacts_[ic]);
            int partIDB = std::get<1>(contacts_[ic]);

            Eigen::Vector3d n = std::get<2>(contacts_[ic]);
            Eigen::Vector3d r = std::get<3>(contacts_[ic]);
            Eigen::Vector3d t1, t2;
            computeFrictionDir(n, t1, t2);

            std::vector<int> partIDs = {partIDA, partIDB};
            std::vector<Eigen::Vector3d> drts;

            //every vector in drts has a corresponding force variable
            if (tension) drts = {n, -n, t1, t2};
            else drts = {n, t1, t2};

            for (int iP = 0; iP < 2; iP++)
            {
                int sign = (iP == 0 ? -1 : 1);
                int partID = partIDs[iP];

                //force
                for (int jdim = 0; jdim < 3; jdim++)
                {
                    int row = partID * 6 + jdim;
                    for (int kdir = 0; kdir < drts.size(); kdir++)
                    {
                        int col = drts.size() * ic + kdir;
                        Eigen::Vector3d fdir = drts[kdir];
                        equlibrium_mat_triplets_.push_back(Eigen::Triplet(row, col, fdir[jdim] * sign));
                    }
                }

                //torque
                for (int jdim = 0; jdim < 3; jdim++)
                {
                    int row = partID * 6 + jdim + 3;
                    for (int kdir = 0; kdir < drts.size(); kdir++)
                    {
                        Eigen::Vector3d m = r.cross(drts[kdir]);
                        int col = drts.size() * ic + kdir;
                        equlibrium_mat_triplets_.push_back(Eigen::Triplet(row, col, m[jdim] * sign));
                    }
                }
            }
        }
    }

    void Analyzer::updateGravity()
    {
        gravity_ = Eigen::VectorXd::Zero(n_part() * 6);
        for (int part_id = 0; part_id < n_part(); part_id++)
        {
            Eigen::Vector3d force(0, -mass_[part_id], 0);
            Eigen::Vector3d r = centroid_[part_id];
            Eigen::Vector3d torque = r.cross(force);
            gravity_.segment(part_id * 6, 3) = force;
            gravity_.segment(part_id * 6 + 3, 3) = torque;
        }
    }

    void Analyzer::computeFrictionDir(const Eigen::Vector3d &n, Eigen::Vector3d &t1, Eigen::Vector3d &t2) {
        t1 = n.cross(Eigen::Vector3d(1, 0, 0));
        if (t1.norm() < 1E-4) {
            t1 = n.cross(Eigen::Vector3d(0, 1, 0));
        }
        t1.normalize();
        t2 = n.cross(t1);
        t2.normalize();
    }

    bool Analyzer::checkStability(Eigen::VectorXd &contactForces, bool tension)
    {
        try{
            GRBEnv env = GRBEnv(true);
            env.start();

            GRBModel model = GRBModel(env);

            //contact force
            GRBVar *contactForceVars = createContactForceVars(model, tension);

            //support force
            GRBVar *supportForceVars = createSupportForceVars(model);

            //Equilibrium
            setForceEquilibrium(model, contactForceVars, supportForceVars);

            //Friction Cone
            setFrictionCone(model, contactForceVars);

            //solve
            model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

            GRBVar obj = model.addVar(0, INFINITY, 1, GRB_CONTINUOUS, "obj");
            GRBQuadExpr obj_expr(0);
            for (int icontact = 0; icontact < contacts_.size(); icontact++)
            {
                int fdim = tension ? 4 : 3;
                int fn = icontact * fdim + fdim - 3;
                obj_expr += contactForceVars[fn] * contactForceVars[fn];
            }
            obj_expr -= obj;
            model.addQConstr(obj_expr, GRB_LESS_EQUAL, 0);

            // Optimize model
            model.optimize();

            if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
            {
                if (tension == false || model.getObjective().getValue() < 1E-6) {
                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        } catch (GRBException e) {
            std::cout << "Error code = " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
        } catch (...) {
            std::cout << "Exception during optimization" << std::endl;
        }
        return false;
    }

    GRBVar *Analyzer::createSupportForceVars(GRBModel &model)
    {
        std::vector<double> lb, ub;
        std::vector<char> types;
        std::vector<double> weights;
        std::vector<std::string> names;
        int dim = 6;
        for (int ipart = 0; ipart < n_part(); ipart++)
        {
            for(int jdim = 0; jdim < dim; jdim++)
            {
                if(status_[ipart] != Installed) {
                    lb.push_back(-INFINITY);
                    ub.push_back(INFINITY);
                }
                else {
                    lb.push_back(0);
                    ub.push_back(0);
                }
                weights.push_back(0);
                types.push_back(GRB_CONTINUOUS);
                names.push_back("support_forces" + std::to_string(ipart * dim + jdim));
            }
        }
        return model.addVars(lb.data(), ub.data(), weights.data(), types.data(), names.data(), lb.size());
    }


    GRBVar *Analyzer::createContactForceVars(GRBModel &model, bool tension)
    {
        std::vector<double> lb, ub;
        std::vector<char> types;
        std::vector<double> weights;
        std::vector<std::string> names;

        int fdim = tension == true? 4 : 3;

        for (int icontact = 0; icontact < contacts_.size(); icontact++)
        {
            int partIDA = std::get<0>(contacts_[icontact]);
            int partIDB = std::get<1>(contacts_[icontact]);
            for (int jforce = 0; jforce < fdim; jforce++)
            {
                if(status_[partIDA] == Uninstalled || status_[partIDB] == Uninstalled) {
                    lb.push_back(0);
                    ub.push_back(0);
                }
                else if (jforce < fdim - 2)
                {
                    lb.push_back(0);
                    ub.push_back(INFINITY);
                }
                else {
                    lb.push_back(-INFINITY);
                    ub.push_back(INFINITY);
                }
                weights.push_back(0);
                types.push_back(GRB_CONTINUOUS);
                names.push_back("forces" + std::to_string(icontact * fdim + jforce));
            }
        }
        return model.addVars(lb.data(), ub.data(), weights.data(), types.data(), names.data(), lb.size());
    }

    void Analyzer::setForceEquilibrium(GRBModel &model, GRBVar *contactForces, GRBVar *supportForces)
    {
        std::vector<GRBLinExpr> exprs;
        exprs.resize(6 * n_part(), GRBLinExpr(0));
        for (int id = 0; id < equlibrium_mat_triplets_.size(); id++)
        {
            int row = equlibrium_mat_triplets_[id].row();
            int col = equlibrium_mat_triplets_[id].col();
            exprs[row] += equlibrium_mat_triplets_[id].value() * contactForces[col];
        }

        int dim = 6;
        for(int ipart; ipart < n_part(); ipart++) {
            for(int jdim; jdim < dim; jdim ++) {
                int row = ipart * dim + jdim;
                exprs[row] += supportForces[row] + gravity_[row];
                model.addConstr(exprs[row], GRB_EQUAL, 0.0);
            }
        }
    }

    void Analyzer::setFrictionCone(GRBModel &model, GRBVar *forceVars, bool tension)
    {
        //friction
        int fdim = tension ? 4: 3;
        for (int icontact = 0; icontact < contacts_.size(); icontact++)
        {
            int fn = icontact * fdim;
            int fr_t1 = icontact * fdim + fdim - 2;
            int fr_t2 = icontact * fdim + fdim - 1;
            model.addConstr(forceVars[fr_t1] - friction_mu_ * forceVars[fn], GRB_LESS_EQUAL, 0.0);
            model.addConstr(forceVars[fr_t1] + friction_mu_ * forceVars[fn], GRB_GREATER_EQUAL, 0.0);
            model.addConstr(forceVars[fr_t2] - friction_mu_ * forceVars[fn], GRB_LESS_EQUAL, 0.0);
            model.addConstr(forceVars[fr_t2] + friction_mu_ * forceVars[fn], GRB_GREATER_EQUAL, 0.0);
        }
    }
}
