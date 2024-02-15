//
// Created by Ziqi Wang on 12.02.2024.
//

#ifndef SURFACE_MESH_SELECT_H
#define SURFACE_MESH_SELECT_H

#include "rigid_block/Analyzer.h"
#include "polyscope/surface_mesh.h"

namespace polyscope
{
    static float radius_ratio = 1;
    static float arrow_length_ratio = 1;
    static std::vector<std::string> arrows_name;
    static std::vector<int> arrows_visible;

    class SurfaceMeshCustomSelect: public polyscope::SurfaceMesh
    {
    private:

        std::vector<CurveNetwork*> arrows_render_obj_;

        float radius_ = 0;

        std::vector<rigid_block::Arrow> arrows_data;

    public:

        SurfaceMeshCustomSelect(std::string name, const std::vector<glm::vec3>& vertexPositions,
            const std::vector<uint32_t>& faceIndsEntries, const std::vector<uint32_t>& faceIndsStart): SurfaceMesh(name, vertexPositions, faceIndsEntries, faceIndsStart){

        }

    public:
        void registerArrows(std::vector<rigid_block::Arrow> &arrows)
        {
            arrows_data = arrows;
            arrows_render_obj_.clear();
            arrows_visible.clear();
            arrows_name.clear();
            auto group = polyscope::getGroup("arrows");

            for(auto arrow: arrows)
            {
                auto curve = polyscope::registerCurveNetwork(name + "_" + arrow.name, arrow.V, arrow.E);
                curve->setRadius(radius_ * radius_ratio, false);
                curve->setEnabled(false);
                curve->setColor({arrow.color.x(), arrow.color.y(), arrow.color.z()});

                curve->addToGroup(*group);

                arrows_name.push_back(arrow.name);
                arrows_visible.push_back(true);
                arrows_render_obj_.push_back(curve);
            }
        }

        void set_radius(double radius) {
            radius_ = radius;
        }

        std::string typeName() override {
            return "Block Mesh";
        }

        void updateArrowLength() {
            auto group = polyscope::getGroup("assembly");
            for (int partID = 0; partID < group->childrenStructures.size(); partID ++)
            {
                SurfaceMeshCustomSelect * surface = (SurfaceMeshCustomSelect *)(&group->childrenStructures[partID].get());
                for(int arrow_id = 0; arrow_id < surface->arrows_data.size(); arrow_id++)
                {
                    Eigen::MatrixXd V = surface->arrows_data[arrow_id].V;
                    for(int ir = 0; ir < V.rows() / 2; ir++) {
                        Eigen::Vector3d p0 = V.row(2 * ir);
                        Eigen::Vector3d p1 = V.row(2 * ir + 1);
                        V.row(2 * ir + 1) = (p1 - p0) * arrow_length_ratio + p0;
                    }
                    surface->arrows_render_obj_[arrow_id]->updateNodePositions(V);
                }
            }
        }

        void buildPickUI(size_t localPickID) override
        {
            static float transparency = 0.2;
            auto group = polyscope::getGroup("assembly");
            for (const auto &structure: group->childrenStructures) {
                structure.get().setTransparency(transparency);
            }
            group = polyscope::getGroup("arrows");
            for (const auto &structure: group->childrenStructures) {
                structure.get().setEnabled(false);
            }


            if(ImGui::CollapsingHeader("Render", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if(ImGui::SliderFloat("Transparency", &transparency, 0.0, 1.0, "%.2f")) {

                }

                if(ImGui::SliderFloat("Radius", &radius_ratio, 0.0, 2.0, "%.3f")) {

                }

                if(ImGui::SliderFloat("Length", &arrow_length_ratio, 0.0, 2.0, "%.3f")) {
                    updateArrowLength();
                }
            }

            if(ImGui::CollapsingHeader("Forces", ImGuiTreeNodeFlags_DefaultOpen))
            {
                for(int id = 0; id < arrows_render_obj_.size(); id++)
                {
                    ImGui::Checkbox(arrows_name[id].c_str(), (bool *)(&arrows_visible[id]));
                    arrows_render_obj_[id]->setEnabled(arrows_visible[id]);
                    arrows_render_obj_[id]->setRadius(radius_ * radius_ratio);
                }
            }

            polyscope::refresh();
        }
    };

    SurfaceMeshCustomSelect* registerSurfaceMeshCustomSelect(std::string name, const Eigen::MatrixXd& vertexPositions, const Eigen::MatrixXi& faceIndices, double avg_length)
    {
        checkInitialized();

        std::tuple<std::vector<uint32_t>, std::vector<uint32_t>> nestedListTup =
            standardizeNestedList<uint32_t, uint32_t, Eigen::MatrixXi>(faceIndices);

        std::vector<uint32_t>& faceIndsEntries = std::get<0>(nestedListTup);
        std::vector<uint32_t>& faceIndsStart = std::get<1>(nestedListTup);

        SurfaceMeshCustomSelect* s =
            new SurfaceMeshCustomSelect(name, standardizeVectorArray<glm::vec3, 3>(vertexPositions), faceIndsEntries, faceIndsStart);

        s->set_radius(avg_length * 2e-3);

        bool success = registerStructure(s);
        if (!success) {
            safeDelete(s);
        }

        return s;
    }
}

#endif //SURFACE_MESH_SELECT_H
