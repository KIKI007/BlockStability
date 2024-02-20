//
// Created by Ziqi Wang on 19.02.2024.
//

#include "render/BlockSelect.h"

void render::BlockSelect::registerArrows(std::vector<rigid_block::Arrow> &arrows) {
    arrows_data = arrows;
    arrows_render_obj_.clear();
    arrows_visible.clear();
    arrows_name.clear();
    for (auto arrow: arrows) {
        auto curve = polyscope::registerCurveNetwork(name + "_" + arrow.name, arrow.V, arrow.E);
        curve->setRadius(arrow_radius_ * radius_ratio, false);
        curve->setEnabled(false);
        curve->setColor({arrow.color.x(), arrow.color.y(), arrow.color.z()});
        curve->addToGroup(*arrow_group_);
        arrows_name.push_back(arrow.name);
        arrows_visible.push_back(true);
        arrows_render_obj_.push_back(curve);
    }
}

void render::BlockSelect::updateArrowLength()
{
    for (int partID = 0; partID < surface_group_->childrenStructures.size(); partID++) {
        BlockSelect *surface = (BlockSelect *) (&surface_group_->childrenStructures[partID].get());
        for (int arrow_id = 0; arrow_id < surface->arrows_data.size(); arrow_id++) {
            Eigen::MatrixXd V = surface->arrows_data[arrow_id].V;
            for (int ir = 0; ir < V.rows() / 2; ir++) {
                Eigen::Vector3d p0 = V.row(2 * ir);
                Eigen::Vector3d p1 = V.row(2 * ir + 1);
                V.row(2 * ir + 1) = (p1 - p0) * arrow_length_ratio + p0;
            }
            surface->arrows_render_obj_[arrow_id]->updateNodePositions(V);
        }
    }
    polyscope::refresh();
}

void render::BlockSelect::buildPickUI(size_t localPickID) {
    static float transparency = 0.2;
    for (const auto &structure: surface_group_->childrenStructures) {
        structure.get().setTransparency(transparency);
    }
    for (const auto &structure: arrow_group_->childrenStructures) {
        structure.get().setEnabled(false);
    }

    if (ImGui::CollapsingHeader("Render", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::SliderFloat("Transparency", &transparency, 0.0, 1.0, "%.2f")) {
        }

        if (ImGui::SliderFloat("Radius", &radius_ratio, 0.0, 2.0, "%.3f")) {
        }

        if (ImGui::SliderFloat("Length", &arrow_length_ratio, 0.0, 2.0, "%.3f")) {
            updateArrowLength();
        }
    }

    if (ImGui::CollapsingHeader("Forces", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (int id = 0; id < arrows_render_obj_.size(); id++) {
            ImGui::Checkbox(arrows_name[id].c_str(), (bool *) (&arrows_visible[id]));
            arrows_render_obj_[id]->setEnabled(arrows_visible[id]);
            arrows_render_obj_[id]->setRadius(arrow_radius_ * radius_ratio);
        }
    }
    polyscope::refresh();
}

