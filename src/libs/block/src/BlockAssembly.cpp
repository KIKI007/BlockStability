//
// Created by 汪子琦 on 04.09.22.
//

#include "block/BlockAssembly.h"
#include "block/PolyPolyBoolean.h"
#include "block/ConvexHull2D.h"
namespace block
{
void BlockAssembly::computeContacts(const std::vector<int> &subPartIDs, std::vector<Contact> &contacts)
{
    contacts.clear();
    for(int id = 0; id < subPartIDs.size(); id++)
    {
        for(int jd = id + 1; jd < subPartIDs.size(); jd++)
        {
            int partIDA = subPartIDs[id];
            int partIDB = subPartIDs[jd];
            std::vector<block::Contact> list_contacts;
            computeContacts(blocks_[partIDA], blocks_[partIDB], list_contacts);
            for(int kd = 0; kd < list_contacts.size(); kd++){
                list_contacts[kd].partIDA = id;
                list_contacts[kd].partIDB = jd;
            }
            contacts.insert(contacts.end(), list_contacts.begin(), list_contacts.end());
        }
    }
}

void BlockAssembly::computeContacts(std::shared_ptr<Block> block1,
                                    std::shared_ptr<Block> block2,
                                    std::vector<block::Contact>& contacts)
{
    PolyPolyBoolean boolean;

    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> normals;

    for(int id = 0; id < block1->F_.rows(); id++)
    {
        Eigen::Vector3d n1 = block1->normal(id);
        Eigen::Vector3d p1 = block1->center(id);
        for(int jd = 0; jd < block2->F_.rows(); jd++)
        {
            Eigen::Vector3d n2 = block2->normal(jd);
            Eigen::Vector3d p2 = block2->center(jd);
//            std::cout << (n1 + n2).transpose() << std::endl;

            if((n1 + n2).norm() <= error_small_normal_
                && abs((p2 - p1).dot(n1)) <= error_small_distance_)
            {
                PolyPolyBoolean::PolyVector3 poly1 = block1->face(id);
                PolyPolyBoolean::PolyVector3 poly2 = block2->face(jd);
                PolyPolyBoolean::PolysVector3 outs;
                boolean.computePolygonsIntersection(poly1, poly2, outs);

                for(PolyPolyBoolean::PolyVector3 out : outs){
                    for(int kd = 0; kd < out.size(); kd++)
                    {
                        points.push_back(out[kd]);
                        normals.push_back(n1);
                    }
                }
            }
        }
    }
    simplifyContact(points, normals, contacts);
    return ;
}

void BlockAssembly::simplifyContact(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &normals, std::vector<Contact> &outputs)
{
    std::vector<int> groupIDs;
    groupIDs.resize(points.size(), -1);
    int count = 0;
    for(int id = 0; id < points.size(); id++)
    {
        if(groupIDs[id] == -1)
        {
            Eigen::Vector3d pi = points[id];
            Eigen::Vector3d ni = normals[id];
            groupIDs[id] = count++;
            for(int jd = id + 1; jd < points.size(); jd++){
                if(groupIDs[jd] == -1)
                {
                    Eigen::Vector3d pj = points[jd];
                    Eigen::Vector3d nj = normals[jd];
                    if((ni - nj).norm() < error_small_normal_
                        && abs((pi - pj).dot(ni)) < error_small_distance_){
                        groupIDs[jd] = groupIDs[id];
                    }
                }
            }
        }
    }

    ConvexHull2D<double> convexhull;
    for(int id = 0; id < count; id++)
    {
        std::vector<Eigen::Vector3d> hull_pts;
        Eigen::Vector3d hull_n;
        for(int jd = 0; jd < points.size(); jd++){
            if(groupIDs[jd] == id){
                hull_pts.push_back(points[jd]);
                hull_n = normals[jd];
            }
        }
        std::vector<Eigen::Vector3d> hull;
        convexhull.compute(hull_pts, hull_n, hull);
        Contact newContact;
        if(!hull.empty())
        {
            std::vector<Eigen::Vector3d> results;
            results.push_back(hull.front());
            for(int jd = 1; jd < hull.size(); jd++)
            {
                Eigen::Vector3d pt = hull[jd];
                if(results.size() == 1){
                    if((results[0] - pt).norm() > error_small_distance_){
                        results.push_back(pt);
                    }
                }
                else if(results.size() == 2){
                    Eigen::Vector3d p0 = results[0];
                    Eigen::Vector3d p1 = results[1];
                    Eigen::Vector3d p01 = p1 - p0;
                    Eigen::Vector3d p1t = pt - p1;
                    if(p01.cross(p1t).norm() > error_small_normal_){
                        results.push_back(pt);
                    }
                    else{
                        results[1] = pt;
                    }
                }
                else
                {
                    Eigen::Vector3d p0 = results[0];
                    Eigen::Vector3d p1 = results[1];
                    Eigen::Vector3d p2 = results[results.size() - 2];
                    Eigen::Vector3d p3 = results[results.size() - 1];
                    Eigen::Vector3d p23 = p3 - p2;
                    Eigen::Vector3d p3t = pt - p3;
                    Eigen::Vector3d pt0 = p0 - pt;
                    Eigen::Vector3d p01 = p1 - p0;

                    if(p3t.cross(pt0).norm() < error_small_normal_){
                        continue ;
                    }
                    else{
                        if(p23.cross(p3t).norm() < error_small_normal_){
                            results.back() = pt;
                            continue ;
                        }

                        if(pt0.cross(p01).norm() < error_small_normal_){
                            results.front() = pt;
                            continue ;
                        }
                    }
                    results.push_back(pt);
                }
            }
            newContact.points = results;
            newContact.normal = hull_n;
            outputs.push_back(newContact);
        }
    }
    return;
}

void BlockAssembly::loadFromFile(std::string filename) {
    readOBJ readObj;
    readObj.loadFromFile(filename);
    for(int id = 0; id < readObj.Vs_.size(); id++){
        std::shared_ptr<Block> block = std::make_shared<Block>();
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

void BlockAssembly::updateGroundBlocks() {
    for(int id = 1; id < blocks_.size(); id++){
        std::shared_ptr<Block> block = blocks_[id - 1];
        std::shared_ptr<Block> ground = blocks_.back();
        std::vector<Contact> contacts;
        computeContacts(block, ground, contacts);
        if(!contacts.empty()){
            block->ground_ = true;
        }
    }
}

void BlockAssembly::addGroundPlane()
{
    Eigen::Vector3d minCoord, maxCoord;
    for(int id = 0; id < blocks_.size(); id++)
    {
        for(int kd = 0; kd < 3; kd++)
        {
            Eigen::VectorXd col = blocks_[id]->V_.col(kd);

            if(id == 0) minCoord[kd] = col.minCoeff();
            else minCoord[kd] = std::min(col.minCoeff(), minCoord[kd]);

            if(id == 0) maxCoord[kd] = col.maxCoeff();
            else maxCoord[kd] = std::max(col.maxCoeff(), maxCoord[kd]);
        }
    }

    double height = minCoord[1];

    Eigen::Vector3d center = (minCoord + maxCoord) / 2;
    Eigen::Vector3d size = (maxCoord - minCoord) / 2;
    size *= 1.5;
    minCoord = center - size;
    maxCoord = center + size;

    std::shared_ptr<Block> block = std::make_shared<Block>();
    block->V_ = Eigen::MatrixXd(4, 3);
    block->V_ << minCoord[0], height, minCoord[2],
                 minCoord[0], height, maxCoord[2],
                 maxCoord[0], height, maxCoord[2],
                 maxCoord[0], height, minCoord[2];
    block->F_ = Eigen::MatrixXi(2, 3);
    block->F_ << 0, 1, 2,
                 0, 2, 3;
    block->N_ = Eigen::MatrixXd(4, 3);
    block->N_ << 0, 1, 0,
        0, 1, 0,
        0, 1, 0,
        0, 1, 0;
    block->partID_ = blocks_.size();
    block->ground_ = true;
    blocks_.push_back(block);
}

std::shared_ptr<block_stability::ContactGraphForce> BlockAssembly::computeContactGraph(std::vector<int> &subPartIDs)
{
    std::sort(subPartIDs.begin(), subPartIDs.end());
    std::shared_ptr<block_stability::ContactGraphForce> graph = std::make_shared<block_stability::ContactGraphForce>(subPartIDs.size());
    graph->density_ = density_;

    std::vector<int> fixedPartIDs;
    for(int id = 0; id < subPartIDs.size(); id++)
    {
        int partID = subPartIDs[id];
        graph->setPart3(id, blocks_[partID]->volume(), blocks_[partID]->centroid());
        if(blocks_[partID]->ground_){
            fixedPartIDs.push_back(id);
        }
    }

    std::vector<Contact> contacts;
    computeContacts(subPartIDs, contacts);

    for(int id = 0; id < contacts.size(); id++)
    {
        graph->addContact3(contacts[id].partIDA, contacts[id].partIDB, contacts[id].normal, contacts[id].points);
    }

    graph->setFixedParts(fixedPartIDs);
    return graph;
}

bool BlockAssembly::checkStability(vector<int> &subPartIDs, Eigen::VectorXd &force)
{
    std::shared_ptr<block_stability::ContactGraphForce> graph = computeContactGraph(subPartIDs);
    try{
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "mip1.log");
        env.set(GRB_IntParam_OutputFlag, 1);
        env.start();

        GRBModel model = GRBModel(env);

        //force
        GRBVar* forces = computeForceVars(model, graph);

        //Support
        GRBVar* supportVars = computeSupportVars(model, graph);

        //Equilibrium
        std::vector<GRBLinExpr> exprs;
        exprs.resize(6 * graph->n_dynamic_part(), GRBLinExpr(0));
        appendInternalForces(model, graph, forces, exprs);
        Eigen::VectorXd w;
        graph->computeGravity(w);
        for(int id = 0; id < exprs.size(); id++)
        {
            exprs[id] += w(id) + supportVars[id];
            model.addConstr(exprs[id], GRB_EQUAL, 0.0);
        }

        //Friction Cone
        setFrictionCone(model, graph, forces);

        //solve
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        GRBVar obj = model.addVar(0, INFINITY, 1, GRB_CONTINUOUS, "obj");
        GRBQuadExpr obj_expr(0);
        for(int id = 0; id < exprs.size(); id++){
            obj_expr += supportVars[id] * supportVars[id];
        }
        obj_expr -= obj;
        model.addQConstr(obj_expr, GRB_LESS_EQUAL, 0);

        // Optimize model
        model.optimize();

        if(model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
        {
            support_forces_.clear();
            support_forces_.resize(blocks_.size(), Eigen::Vector3d(0, 0, 0));
            for(int id = 0; id < graph->n_dynamic_part_; id++)
            {
                Eigen::Vector3d force(0, 0, 0);
                for(int jd = 0; jd < 6; jd++)
                {
                    int varID = id * 6 + jd;
                    double f = supportVars[varID].get(GRB_DoubleAttr_X);
//                    std::cout << f << " ";
                    if(jd < 3) force[jd] = f;
                }
                int oldID = graph->mapNewToOld_[id];
                support_forces_[oldID] = force;
//                std::cout << std::endl;
            }
            if(model.getObjective().getValue() < 1E-6){
                return true;
            }
            else{
                return false;
            }

        }
        else{
            return false;
        }
    }
    catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch (...) {
        std::cout << "Exception during optimization" << std::endl;
    }
    return false;
}

bool BlockAssembly::checkStability(std::vector<int> &subPartIDs)
{
    Eigen::VectorXd force;
    return checkStability(subPartIDs, force);
}

GRBVar *BlockAssembly::computeForceVars(GRBModel &model, std::shared_ptr<block_stability::ContactGraphForce> &graph)
{
    GRBVar *forceVars;
    std::vector<double> lb, ub;
    std::vector<char> types;
    std::vector<double> weights;
    std::vector<std::string> names;
    for (int id = 0; id < graph->n_dynamic_contact(); id++)
    {
        for(int jd = 0; jd < 3; jd++)
        {
            if (jd == 0)
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
            names.push_back("forces" + std::to_string(id * 3 + jd));
        }
    }
    forceVars = model.addVars(lb.data(), ub.data(), weights.data(), types.data(), names.data(), lb.size());
    return forceVars;
}

void BlockAssembly::appendInternalForces(GRBModel &model, std::shared_ptr<block_stability::ContactGraphForce> &graph, GRBVar *force, vector<GRBLinExpr> &exprs)
{
    std::vector<Eigen::Triplet<double>> A;
    graph->computeForceMatrix(A, false);

    for(int id = 0; id < A.size(); id++){
        int row = A[id].row();
        int col = A[id].col();
        exprs[row] += A[id].value() * force[col];
    }
}

void BlockAssembly::setFrictionCone(GRBModel &model, std::shared_ptr<block_stability::ContactGraphForce> &graph, GRBVar *force)
{
    //friction
    for(int id = 0; id < graph->n_dynamic_contact(); id++)
    {
        int fn = id * 3;
        int fr_t1 = id * 3 + 1;
        int fr_t2 = id * 3 + 2;
        model.addConstr(force[fr_t1] - friction_mu_ * force[fn], GRB_LESS_EQUAL, 0.0);
        model.addConstr(force[fr_t1] + friction_mu_ * force[fn] , GRB_GREATER_EQUAL, 0.0);
        model.addConstr(force[fr_t2] - friction_mu_ * force[fn], GRB_LESS_EQUAL, 0.0);
        model.addConstr(force[fr_t2] + friction_mu_ * force[fn], GRB_GREATER_EQUAL, 0.0);
    }
}

    GRBVar * BlockAssembly::computeSupportVars(GRBModel &model, std::shared_ptr<block_stability::ContactGraphForce> &graph) {
        GRBVar *supportVars;
        std::vector<double> lb, ub;
        std::vector<char> types;
        std::vector<double> weights;
        std::vector<std::string> names;
        for (int id = 0; id < graph->n_dynamic_part_ * 6; id++)
        {
            if(id % 6 < 3){
                lb.push_back(-INFINITY);
                ub.push_back(INFINITY);
            }
            else{
                lb.push_back(0);
                ub.push_back(0);
            }
            types.push_back(GRB_CONTINUOUS);
            weights.push_back(0);
            names.push_back("support" + std::to_string(id));
        }
        supportVars = model.addVars(lb.data(), ub.data(), weights.data(), types.data(), names.data(), lb.size());
        return supportVars;
    }
}

