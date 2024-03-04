//
// Created by Ziqi Wang on 19.02.2024.
//

#include "render/AssemblyRender.h"

#include <sys/stat.h>

render::AssemblyRender::AssemblyRender(std::string name, std::shared_ptr<rigid_block::Assembly> assembly) {
    name_ = name;
    assembly_ = assembly;
    for(int id = 0; id < assembly_->blocks_.size(); id++) {
        installed_part_ids_.push_back(id);
    }
    create();
}

render::AssemblyRender::~AssemblyRender()
{
    for(auto obj: blocks_objs_) {
        polyscope::removeStructure(obj);
    }

    polyscope::removeGroup(assembly_group_);
    polyscope::removeGroup(arrow_group_);
    polyscope::removeStructure(contact_mesh_);
    polyscope::removeStructure(ground_mesh_);
}

void render::AssemblyRender::compute()
{
    analyzer_->updateFrictionCeoff(friction_coeff_);
    rigid_block::AnalysisResult result;
    auto status = getStatus();

    double val = analyzer_->solve(status, result);
    polyscope::info("Stability: \t " + std::to_string(val));

    for(int partID = 0; partID < assembly_group_->childrenStructures.size(); partID++)
    {
        std::vector<rigid_block::Arrow> arrows = result.computeArrows(partID);
        auto obj = (BlockSelect *)(&assembly_group_->childrenStructures[partID].get());
        obj->registerArrows(arrows);
    }

    arrow_group_->setHideDescendantsFromStructureLists(true);
    arrow_group_->setShowChildDetails(false);
}

void render::AssemblyRender::update()
{
    assembly_group_->setEnabled(show_assembly_);

    if(show_assembly_)
        {
        for(int partID = 0; partID < assembly_group_->childrenStructures.size(); partID++)
        {
            auto obj = (BlockSelect *)(&assembly_group_->childrenStructures[partID].get());
            if(assembly_->blocks_[partID]->ground_) {
                obj->setEnabled(true);
                obj->setSurfaceColor(glm::vec3(0, 0, 0));
            }
            else if(std::find(held_part_ids_.begin(), held_part_ids_.end(), partID) != held_part_ids_.end()) {
                obj->setEnabled(true);
                obj->setSurfaceColor(glm::vec3(0.3, 0.3, 0.3));
            }
            else if(std::find(installed_part_ids_.begin(), installed_part_ids_.end(), partID) != installed_part_ids_.end()) {
                obj->setEnabled(true);
                obj->setSurfaceColor(glm::vec3(1, 1, 1));
            }
            else {
                obj->setEnabled(false);
            }
        }
    }

    contact_mesh_->setEnabled(show_contact_);
    ground_mesh_->setEnabled(show_ground_);
}

void render::AssemblyRender::create()
{
    //analyzer
    analyzer_ = assembly_->createAnalyzer(false);

    //create group
    assembly_group_ = polyscope::createGroup(name_ + "_Assembly");
    arrow_group_ = polyscope::createGroup(name_ + "_Arrows");

    //create blocks
    double avg_length = assembly_->computeAvgDiagnalLength();
    for (int id = 0; id < assembly_->blocks_.size(); id++) {
        std::string blockname = name_ + "_Block" + std::to_string(id);
        auto block = assembly_->blocks_[id];
        auto block_obj = render::registerBlockSelect(blockname, block->V_, block->F_, avg_length);
        Eigen::Vector3d color = block->color();
        block_obj->setSurfaceColor({color[0], color[1], color[2]});
        block_obj->setEnabled(show_assembly_);
        block_obj->addToGroup(*assembly_group_);
        block_obj->surface_group_ = assembly_group_;
        block_obj->arrow_group_ = arrow_group_;
        blocks_objs_.push_back(block_obj);
    }
    arrow_group_->setHideDescendantsFromStructureLists(true);
    arrow_group_->setShowChildDetails(false);

    //create contacts
    {
        auto contacts = assembly_->computeContacts(installed_part_ids_);

        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        assembly_->toMesh(contacts, V, F);

        contact_mesh_ = polyscope::registerSurfaceMesh(name_ + "_Contact", V, F);
        contact_mesh_->setEnabled(show_contact_);
    }

    //create ground
    {
        std::vector<rigid_block::ContactFace> contacts;
        for (int ipart : installed_part_ids_)
            {
            std::vector<rigid_block::ContactFace> ground_contacts
                    = assembly_->computeContacts(assembly_->ground_plane_, assembly_->blocks_[ipart]);
            contacts.insert(contacts.end(), ground_contacts.begin(), ground_contacts.end());
        }

        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        assembly_->toMesh(contacts, V, F);

        ground_mesh_ = polyscope::registerSurfaceMesh(name_ + "_Ground", V, F);
        ground_mesh_->setEnabled(show_ground_);
    }
}

std::vector<rigid_block::Analyzer::PartStatus> render::AssemblyRender::getStatus() {
    std::vector<rigid_block::Analyzer::PartStatus> status;
    for(int partID = 0; partID < assembly_group_->childrenStructures.size(); partID++)
    {
        if(assembly_->blocks_[partID]->ground_ || std::find(held_part_ids_.begin(), held_part_ids_.end(), partID) != held_part_ids_.end()) {
            status.push_back(rigid_block::Analyzer::Fixed);
        }
        else if(std::find(installed_part_ids_.begin(), installed_part_ids_.end(), partID) != installed_part_ids_.end()) {
            status.push_back(rigid_block::Analyzer::Installed);
        }
        else {
            status.push_back(rigid_block::Analyzer::Uninstalled);
        }
    }
    return status;
}

void render::AssemblyRender::mouseEvent() {
    ImGuiIO &io = ImGui::GetIO();
    if (io.MouseClicked[0])
    {
        // if the left mouse button was clicked
        // gather values
        glm::vec2 screenCoords{io.MousePos.x, io.MousePos.y};
        std::pair<polyscope::Structure *, size_t> pickPair =
                polyscope::pick::evaluatePickQuery(screenCoords.x * 2, screenCoords.y * 2);

        if (pickPair.first == nullptr)
        {
            for (const auto &structure: assembly_group_->childrenStructures) {
                structure.get().setTransparency(1.0);
            }
            for (const auto &structure: arrow_group_->childrenStructures) {
                structure.get().setEnabled(false);
            }
        }
    }
}

void render::AssemblyRender::gui()
{
    std::string headname = name_ + "_gui";
    if (ImGui::CollapsingHeader(headname.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {

        ImGui::SliderFloat("friction", &friction_coeff_, 0, 1);

        if (ImGui::Checkbox("assembly", &show_assembly_)) {
            update();
        }

        if (ImGui::Checkbox("contacts within block", &show_contact_)) {
            update();
        }

        if (ImGui::Checkbox("contacts on ground", &show_ground_)) {
            update();
        }
    }
}
