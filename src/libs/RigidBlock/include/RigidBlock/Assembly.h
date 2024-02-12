//
// Created by 汪子琦 on 04.09.22.
//

#ifndef ROBO_CRAFT_BLOCKASSEMBLY_H
#define ROBO_CRAFT_BLOCKASSEMBLY_H

#include "Part.h"
#include "util/readOBJ.h"
#include "Analyzer.h"
#include "gurobi_c++.h"

namespace rigid_block
{
    class Contact
    {
    public:
        int partIDA;
        int partIDB;
        std::vector<Eigen::Vector3d> points;
        Eigen::Vector3d normal;
    };

    class Assembly
    {
    public:
        std::vector<std::shared_ptr<Part> > blocks_;

        std::shared_ptr<Part> ground_plane_;

        const double error_small_normal_ = 1E-4;

        const double error_small_distance_ = 1E-2;

        double friction_mu_;

    public:

        Assembly()
        {
        }

        Assembly(std::vector<std::shared_ptr<Part> > blocks): blocks_(blocks) {
        }

    public:

        void loadFromFile(std::string filename);

        std::vector<Contact> computeContacts(const std::vector<int> &subPartIDs);

        std::vector<Contact> computeContacts(std::shared_ptr<Part> block1, std::shared_ptr<Part> block2);

        std::vector<Contact> simplifyContact(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &normals);

        void addGroundPlane();

        void updateGroundBlocks();

        std::shared_ptr<block_stability::Analyzer> createAnalyzer();
    };
}


#endif  //ROBO_CRAFT_BLOCKASSEMBLY_H
