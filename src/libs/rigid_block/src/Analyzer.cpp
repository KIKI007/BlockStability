//
// Created by 汪子琦 on 05.09.22.
//

#include "rigid_block/Analyzer.h"
#include <iostream>

namespace rigid_block
{
    void Arrow::from_points(const std::vector<Eigen::Vector3d> &points) {
        V = Eigen::MatrixXd(points.size(), 3);
        E = Eigen::MatrixXi(points.size() / 2, 2);
        for(int id = 0; id < points.size() / 2; id++) {
            V.row(2 * id) = points[id * 2];
            V.row(2 * id + 1) = points[id * 2 + 1];
            E.row(id) = Eigen::RowVector2i(2 * id, 2 * id + 1);
        }
    }

    std::vector<Arrow> AnalysisResult::computeArrows(int partID)
    {

        if(contact_points.empty())
            return {};

        std::vector<Eigen::Vector3d> compression_pts, tension_pts, friction_pts;
        std::vector<Eigen::Vector3d> gravity_pts, support_pts;
        int fdim = with_tension ? 4 : 3;

        for(int icontact = 0; icontact < contact_points.size(); icontact++)
        {
            int partIDA = contact_points[icontact].partIDA;
            int partIDB = contact_points[icontact].partIDB;

            if(partID != partIDA && partID != partIDB) {
                continue;
            }

            Eigen::Vector3d n = contact_points[icontact].contact_normal;
            Eigen::Vector3d r = contact_points[icontact].contact_point;
            Eigen::Vector3d t1 = contact_points[icontact].contact_friction_t1;
            Eigen::Vector3d t2 = contact_points[icontact].contact_friction_t2;

            if(partIDA == partID) {
                n = -n;
                t1 = -t1;
                t2 = -t2;
            }

            //compression
            {
                compression_pts.push_back(r);
                compression_pts.push_back(r + n * internal_contact_forces[icontact * fdim]);
            }

            //tension
            if(with_tension) {
                tension_pts.push_back(r);
                tension_pts.push_back(r - n * internal_contact_forces[icontact * fdim + 1]);
            }

            //friction
            {
                friction_pts.push_back(r);
                friction_pts.push_back(r + t1 * internal_contact_forces[icontact * fdim - 2 + fdim] + t2 * internal_contact_forces[icontact * fdim - 1 + fdim]);
            }
        }

        Eigen::Vector3d ct = centroid[partID];
        Eigen::Vector3d g = gravity_forces.segment(partID * 6, 3);
        Eigen::Vector3d sf = support_forces.segment(partID * 6, 3);
        {
            gravity_pts.push_back(ct);
            gravity_pts.push_back(ct + g);
        }
        {
            support_pts.push_back(ct);
            support_pts.push_back(ct + sf);
        }

        std::vector<Arrow> result;
        {
            Arrow compression;
            compression.from_points(compression_pts);
            compression.name = "compression";
            compression.color = {1.0, 0.0, 0.0};
            result.push_back(compression);
        }


        if(with_tension) {
            Arrow tension;
            tension.from_points(tension_pts);
            tension.name = "tension";
            tension.color = {0.0, 0.0, 1.0};
            result.push_back(tension);
        }

        {
            Arrow friction;
            friction.from_points(friction_pts);
            friction.name = "friction";
            friction.color = {0.0, 1.0, 0.0};
            result.push_back(friction);
        }

        {
            Arrow gravity;
            gravity.from_points(gravity_pts);
            gravity.name = "gravity";
            gravity.color = {1.0, 1.0, 0.0};
            result.push_back(gravity);
        }

        {
            Arrow support;
            support.from_points(support_pts);
            support.name = "support";
            support.color = {0.0, 1.0, 1.0};
            result.push_back(support);
        }

        return result;
    }

    void Analyzer::addContact(int partIDA,
                              int partIDB,
                              const Eigen::Vector3d &normal,
                              const std::vector<Eigen::Vector3d> &normal_point)
    {
        ContactPoint contact_point;
        Eigen::Vector3d t1, t2;
        computeFrictionDir(normal, t1, t2);
        for (int id = 0; id < normal_point.size(); id++)
        {
            contact_point.contact_point = normal_point[id];
            contact_point.contact_normal = normal;
            contact_point.contact_friction_t1 = t1;
            contact_point.contact_friction_t2 = t2;
            contact_point.partIDA = partIDA;
            contact_point.partIDB = partIDB;
            contact_points_.push_back(contact_point);
        }
    }

    void Analyzer::updatePart(int partID, double mass, Eigen::Vector3d ct) {
        if (0 <= partID && partID < n_part()) {
            mass_[partID] = mass;
            centroid_[partID] = ct;
        }
    }

    void Analyzer::updateEquilibriumMatrix()
    {
        for (int ic = 0; ic < contact_points_.size(); ic++)
        {
            int partIDA = contact_points_[ic].partIDA;
            int partIDB = contact_points_[ic].partIDB;

            Eigen::Vector3d n = contact_points_[ic].contact_normal;
            Eigen::Vector3d r = contact_points_[ic].contact_point;
            Eigen::Vector3d t1 = contact_points_[ic].contact_friction_t1;
            Eigen::Vector3d t2 = contact_points_[ic].contact_friction_t2;

            std::vector<int> partIDs = {partIDA, partIDB};
            std::vector<Eigen::Vector3d> drts;

            //every vector in drts has a corresponding force variable
            if (with_tension_) drts = {n, -n, t1, t2};
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
            Eigen::Vector3d force(0, 0, -mass_[part_id]);
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

    double Analyzer::solve(const std::vector<PartStatus> &status, AnalysisResult &result)
    {
        try{
            int fdim = with_tension_ ? 4 : 3;
            int n_contact_force = fdim * n_contact();
            int n_support_force = 6 * n_part();

            GRBEnv env = GRBEnv(true);
            env.set(GRB_IntParam_OutputFlag, 0);
            env.start();

            GRBModel model = GRBModel(env);

            //contact force
            GRBVar *contactForceVars = createContactForceVars(model, status);

            //support force
            GRBVar *supportForceVars = createSupportForceVars(model, status);

            //Equilibrium
            setForceEquilibrium(model, contactForceVars, supportForceVars);

            //Friction Cone
            setFrictionCone(model, contactForceVars);

            //solve
            model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

            GRBVar obj = model.addVar(0, INFINITY, 1, GRB_CONTINUOUS, "obj");
            GRBQuadExpr obj_expr(0);
            for (int icontact = 0; icontact < contact_points_.size(); icontact++)
            {
                int fn = icontact * fdim + fdim - 3;
                obj_expr += contactForceVars[fn] * contactForceVars[fn];
            }
            obj_expr -= obj;
            model.addQConstr(obj_expr, GRB_LESS_EQUAL, 0);

            // Optimize model
            model.optimize();

            if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
            {
                double *v = model.get(GRB_DoubleAttr_X, contactForceVars, n_contact_force);
                result.internal_contact_forces = Eigen::Map<Eigen::VectorXd>(v, n_contact_force);
                v = model.get(GRB_DoubleAttr_X, supportForceVars, n_support_force);
                result.support_forces = Eigen::Map<Eigen::VectorXd>(v, n_support_force);
                result.with_tension = with_tension_;
                result.contact_points = contact_points_;
                result.centroid = centroid_;
                result.gravity_forces = gravity_;
                if (with_tension_ == false)
                {
                    return 0.0;
                }
                return obj.get(GRB_DoubleAttr_X);
            }
            else
            {
                return std::numeric_limits<double>::max();
            }
        } catch (GRBException e) {
            std::cout << "Error code = " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
        } catch (...) {
            std::cout << "Exception during optimization" << std::endl;
        }
        return std::numeric_limits<double>::max();;
    }

    GRBVar *Analyzer::createSupportForceVars(GRBModel &model, const std::vector<PartStatus> &status)
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
                if(status[ipart] != Installed) {
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


    GRBVar *Analyzer::createContactForceVars(GRBModel &model, const std::vector<PartStatus> &status)
    {
        std::vector<double> lb, ub;
        std::vector<char> types;
        std::vector<double> weights;
        std::vector<std::string> names;

        int fdim = (with_tension_ == true? 4 : 3);

        for (int icontact = 0; icontact < contact_points_.size(); icontact++)
        {
            int partIDA = contact_points_[icontact].partIDA;
            int partIDB = contact_points_[icontact].partIDB;
            for (int jforce = 0; jforce < fdim; jforce++)
            {
                if(status[partIDA] == Uninstalled || status[partIDB] == Uninstalled) {
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
        for(int ipart = 0; ipart < n_part(); ipart++) {
            for(int jdim = 0; jdim < dim; jdim ++) {
                int row = ipart * dim + jdim;
                exprs[row] += supportForces[row] + gravity_[row];
                model.addConstr(exprs[row], GRB_EQUAL, 0.0);
            }
        }
    }

    void Analyzer::setFrictionCone(GRBModel &model, GRBVar *forceVars)
    {
        //friction
        int fdim = with_tension_ ? 4: 3;
        for (int icontact = 0; icontact < contact_points_.size(); icontact++)
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
