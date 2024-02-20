//
// Created by Ziqi Wang on 12.02.2024.
//

#ifndef SURFACE_MESH_SELECT_H
#define SURFACE_MESH_SELECT_H

#include "rigid_block/Analyzer.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"

namespace render
{
    static float radius_ratio = 1;
    static float arrow_length_ratio = 1;
    static std::vector<std::string> arrows_name;
    static std::vector<int> arrows_visible;

    class BlockSelect: public polyscope::SurfaceMesh {
    private:
        std::vector<polyscope::CurveNetwork*> arrows_render_obj_;
        std::vector<rigid_block::Arrow> arrows_data;
        float arrow_radius_ = 0;

    public:
        polyscope::Group * surface_group_;
        polyscope::Group * arrow_group_;

    public:

        BlockSelect(std::string name,
            const std::vector<glm::vec3>& vertexPositions,
            const std::vector<uint32_t>& faceIndsEntries,
            const std::vector<uint32_t>& faceIndsStart): SurfaceMesh(name, vertexPositions, faceIndsEntries, faceIndsStart){
        }

        ~BlockSelect() {
            for(auto curve: arrows_render_obj_) {
                polyscope::removeStructure(curve);
            }
        }

    public:
        void registerArrows(std::vector<rigid_block::Arrow> &arrows);

        void setArrowRadius(double radius) {arrow_radius_ = radius;}

        std::string typeName() override {return "Block Mesh";}

        void updateArrowLength();

        void buildPickUI(size_t localPickID) override;
    };

    inline BlockSelect *registerBlockSelect(std::string name, const Eigen::MatrixXd &vertexPositions,
                                                     const Eigen::MatrixXi &faceIndices, double avg_length) {
        polyscope::checkInitialized();

        std::tuple<std::vector<uint32_t>, std::vector<uint32_t> > nestedListTup =
                polyscope::standardizeNestedList<uint32_t, uint32_t, Eigen::MatrixXi>(faceIndices);

        std::vector<uint32_t> &faceIndsEntries = std::get<0>(nestedListTup);
        std::vector<uint32_t> &faceIndsStart = std::get<1>(nestedListTup);

        BlockSelect *s =
                new BlockSelect(name, polyscope::standardizeVectorArray<glm::vec3, 3>(vertexPositions), faceIndsEntries,
                                faceIndsStart);

        s->setArrowRadius(avg_length * 4e-2);

        bool success = registerStructure(s);
        if(!success)
        {
            safeDelete(s);
        }
        return s;
    }
}

#endif //SURFACE_MESH_SELECT_H
