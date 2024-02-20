//
// Created by Ziqi Wang on 19.02.2024.
//

#ifndef ASSEMBLYRENDER_H
#define ASSEMBLYRENDER_H
#include "rigid_block/Assembly.h"
#include "polyscope/pick.h"
#include "BlockSelect.h"

namespace render
{
    class AssemblyRender
    {
    private:



        std::shared_ptr<rigid_block::Assembly> assembly_;

        std::shared_ptr<rigid_block::Analyzer> analyzer_;

    private:

        std::vector<BlockSelect *> blocks_objs_;

        std::string name_;

        polyscope::Group *assembly_group_;

        polyscope::Group *arrow_group_;

        polyscope::SurfaceMesh *contact_mesh_, *ground_mesh_;


    public:

        std::vector<int> installed_part_ids_;

        std::vector<int> held_part_ids_;

        bool show_assembly_ = true;

        bool show_contact_ = false;

        bool show_ground_ = false;

        float friction_coeff_ = 0.5;

    public:

        AssemblyRender(std::string name, std::shared_ptr<rigid_block::Assembly> assembly);

        ~AssemblyRender();
    public:

        void compute();

        void update();

        void create();

        std::vector<rigid_block::Analyzer::PartStatus> getStatus();

        void mouseEvent();

        void gui();

    public:
    };
}

#endif //ASSEMBLYRENDER_H
