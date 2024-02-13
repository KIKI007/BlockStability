#include "RigidBlock/Assembly.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"
#include <imgui.h>
#include "file_dialog_open.h"
#include "polyscope/pick.h"
#include "surface_mesh_custom_select.h"

polyscope::Group *assembly_group = nullptr, *arrow_group = nullptr;
polyscope::SurfaceMesh *contact_render_obj = nullptr, *contact_with_ground_render_obj = nullptr;

static bool show_assembly = true;
static bool show_contact = false;
static bool show_contact_ground = false;
static float fr_coeff = 0.5;
std::shared_ptr<rigid_block::Assembly> blockAssembly;

void renderBlocks(bool visible)
{
    if(assembly_group) {
        polyscope::removeGroup(assembly_group);
    }
    assembly_group = polyscope::createGroup("assembly");
    double avg_length = blockAssembly->computeAvgDiagnalLength();

    for (int id = 0; id < blockAssembly->blocks_.size(); id++) {
        std::string name = "block" + std::to_string(id);
        auto block = blockAssembly->blocks_[id];
        auto block_obj = polyscope::registerSurfaceMeshCustomSelect(name, block->V_, block->F_, avg_length);
        Eigen::Vector3d color = block->color();
        block_obj->setSurfaceColor({color[0], color[1], color[2]});
        block_obj->setEnabled(visible);
        block_obj->addToGroup(*assembly_group);
    }

    assembly_group->setHideDescendantsFromStructureLists(true);
    assembly_group->setShowChildDetails(false);
}

void renderContactsonGround(bool visible) {
    std::vector<rigid_block::ContactFace> contacts;
    for (int ipart = 0; ipart < blockAssembly->blocks_.size(); ipart++) {
        std::vector<rigid_block::ContactFace> ground_contacts
                = blockAssembly->computeContacts(blockAssembly->ground_plane_, blockAssembly->blocks_[ipart]);
        contacts.insert(contacts.end(), ground_contacts.begin(), ground_contacts.end());
    }

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    blockAssembly->toMesh(contacts, V, F);

    contact_with_ground_render_obj = polyscope::registerSurfaceMesh("contact_ground", V, F);
    contact_with_ground_render_obj->setEnabled(visible);
}

void renderContacts(bool visible)
{
    std::vector<int> partIDs;
    for (int id = 0; id < blockAssembly->blocks_.size(); id++) {
        partIDs.push_back(id);
    }
    auto contacts = blockAssembly->computeContacts(partIDs);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    blockAssembly->toMesh(contacts, V, F);

    contact_render_obj = polyscope::registerSurfaceMesh("contact", V, F);
    contact_render_obj->setEnabled(visible);
}

void mouseEventCallback()
{
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
            auto group = polyscope::getGroup("assembly");
            for (const auto &structure: group->childrenStructures) {
                structure.get().setTransparency(1.0);
            }
            group = polyscope::getGroup("arrows");
            for (const auto &structure: group->childrenStructures) {
                structure.get().setEnabled(false);
            }
        }
    }
    polyscope::refresh();
}

void updateRender() {
    polyscope::removeAllStructures();
    renderBlocks(show_assembly);
    renderContacts(show_contact);
    renderContactsonGround(show_contact_ground);
}

void checkStability()
{
    if (blockAssembly)
    {
        blockAssembly->friction_mu_ = fr_coeff;
        auto analyzer = blockAssembly->createAnalyzer();
        rigid_block::AnalysisResult result;

        if (analyzer->solve(result)) {
            polyscope::info("Equilibrium");
        } else {
            polyscope::info("Nonequilibrium");

        }

        if(arrow_group) {
            polyscope::removeGroup(arrow_group);
        }
        arrow_group = polyscope::createGroup("arrows");

        for(int partID = 0; partID < assembly_group->childrenStructures.size(); partID++)
        {
            std::vector<rigid_block::Arrow> arrows = result.computeArrows(partID);
            auto obj = (polyscope::SurfaceMeshCustomSelect *)(&assembly_group->childrenStructures[partID].get());
            obj->registerArrows(arrows);
        }
        arrow_group->setHideDescendantsFromStructureLists(true);
        arrow_group->setShowChildDetails(false);

        auto [bbmin, bbmax] = polyscope::state::boundingBox;
        std::cout << bbmin.y << ", " << bbmax.y << std::endl;

        polyscope::options::groundPlaneHeightFactor = polyscope::absoluteValue(bbmin.y);
    }
}

int main()
{
    polyscope::init();

    assembly_group = polyscope::createGroup("assembly");
    arrow_group = polyscope::createGroup("arrows");
    polyscope::options::groundPlaneHeightFactor = 0;

    // Add content to the default menu window
    polyscope::state::userCallback = [&]()
    {
        mouseEventCallback();

        if (ImGui::CollapsingHeader("UI", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Open File")) {
                std::string obj_file = file_dialog_open();
                blockAssembly = std::make_shared<rigid_block::Assembly>();
                blockAssembly->loadFromFile(obj_file);
                updateRender();
            }

            ImGui::SameLine();

            if (ImGui::Button("Check"))
            {
                checkStability();
            }

            if (ImGui::SliderFloat("friction coeff", &fr_coeff, 0, 1)) {
            }
        }

        if (ImGui::CollapsingHeader("Rendering", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Checkbox("assembly", &show_assembly)) {
                assembly_group->setEnabled(show_assembly);
            }

            if (ImGui::Checkbox("contacts within block", &show_contact)) {
                contact_render_obj->setEnabled(show_contact);
            }

            if (ImGui::Checkbox("contacts on ground", &show_contact_ground)) {
                contact_with_ground_render_obj->setEnabled(show_contact_ground);
            }
        }
    };

    // Give control to the polyscope gui
    polyscope::show();
}
