//
// Created by 汪子琦 on 04.09.22.
//

#ifndef ROBO_CRAFT_BLOCKASSEMBLY_H
#define ROBO_CRAFT_BLOCKASSEMBLY_H

#include "Block.h"
#include "readOBJ.h"
#include "block/ContactGraphForce.h"
#include "gurobi_c++.h"
#include "gurobi_c.h"

namespace block
{

class Contact
{

public:
    int partIDA;
    int partIDB;
    std::vector<Eigen::Vector3d> points;
    Eigen::Vector3d normal;
};

class BlockAssembly
{

public:

    std::vector<std::shared_ptr<Block>> blocks_;
    std::vector<Eigen::Vector3d> support_forces_;

    const double error_small_normal_ = 1E-4;
    const double error_small_distance_ = 1E-2;

    double density_ = 1;

    double friction_mu_ = 0.5;

    double force_ub_ = 1;

public:

    BlockAssembly(){

    }

    BlockAssembly(std::vector<std::shared_ptr<Block>> blocks): blocks_(blocks){

    }

public:

    void loadFromFile(std::string filename);

    void computeContacts(const std::vector<int> &subPartIDs, std::vector<Contact> &contacts);

    void computeContacts(std::shared_ptr<Block> block1, std::shared_ptr<Block> block2, std::vector<block::Contact>& contacts);

    void simplifyContact(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &normals, std::vector<Contact> &outputs);

    void addGroundPlane();

    void updateGroundBlocks();

    std::shared_ptr<block_stability::ContactGraphForce> computeContactGraph(std::vector<int> &subPartIDs);

public:

    bool checkStability(std::vector<int> &subPartIDs);

    bool checkStability(std::vector<int> &subPartIDs, Eigen::VectorXd &force);

    GRBVar* computeForceVars(GRBModel &model, std::shared_ptr<block_stability::ContactGraphForce> &graph);

    GRBVar* computeSupportVars(GRBModel &model, std::shared_ptr<block_stability::ContactGraphForce> &graph);

    void appendInternalForces(GRBModel &model, std::shared_ptr<block_stability::ContactGraphForce> &graph, GRBVar *force, std::vector<GRBLinExpr> &exprs);

    void setFrictionCone(GRBModel &model, std::shared_ptr<block_stability::ContactGraphForce> &graph, GRBVar *force);
};
}


#endif  //ROBO_CRAFT_BLOCKASSEMBLY_H
