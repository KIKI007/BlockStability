#include "rigid_block/Assembly.h"
#include "search/StateGraphHolding.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"
#include "polyscope/pick.h"
#include "search/SearchAlgorithmBeamSearch.h"
std::shared_ptr<rigid_block::Assembly> blockAssembly;
polyscope::Group *assembly_group = nullptr, *arrow_group = nullptr;
int sequence_id = 0;
search::AssemblySequence sequence;
std::vector<int> installed_parts;
std::vector<int> fixed_parts;

void renderBlocks(bool visible)
{
    if(assembly_group) {
        polyscope::removeGroup(assembly_group);
    }
    assembly_group = polyscope::createGroup("assembly");

    for (int id = 0; id < blockAssembly->blocks_.size(); id++) {
        std::string name = "block" + std::to_string(id);
        auto block = blockAssembly->blocks_[id];
        auto block_obj = polyscope::registerSurfaceMesh(name, block->V_, block->F_);
        Eigen::Vector3d color = block->color();
        block_obj->setSurfaceColor({color[0], color[1], color[2]});
        block_obj->setEnabled(visible);
        block_obj->addToGroup(*assembly_group);
    }

    assembly_group->setHideDescendantsFromStructureLists(true);
    assembly_group->setShowChildDetails(false);
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

void renderSequence()
{
    installed_parts.clear();
    fixed_parts.clear();

    for(auto &obj : assembly_group->childrenStructures)
    {
        auto surface = (polyscope::SurfaceMesh *)(&obj.get());
        surface->setEnabled(false);
        surface->setSurfaceColor(glm::vec3(1, 1 ,1));
    }

    for(int id = 0; id < sequence.steps.size() && id < sequence_id + 1; id++)
    {
        auto installPartIDs = sequence.steps[id].installPartIDs_;
        for(auto partID : installPartIDs) {
            auto surface = (polyscope::SurfaceMesh *)(&assembly_group->childrenStructures[partID].get());
            surface->setSurfaceColor(glm::vec3(1, 1 ,1));
            surface->setEnabled(true);
            installed_parts.push_back(partID);
        }
    }

    auto fixedPartIDs = sequence.steps[sequence_id].holdPartIDs_;
    for(auto partID : fixedPartIDs) {
        auto surface = (polyscope::SurfaceMesh *)(&assembly_group->childrenStructures[partID].get());
        if(blockAssembly->blocks_[partID]->ground_)
            surface->setSurfaceColor(glm::vec3(0, 0 ,0));
        else
            surface->setSurfaceColor(glm::vec3(1, 0 ,0));
        fixed_parts.push_back(partID);
    }

    //checkStability();
}


int main() {
    polyscope::init();

    assembly_group = polyscope::createGroup("assembly");
    arrow_group = polyscope::createGroup("arrows");

    blockAssembly = std::make_shared<rigid_block::Assembly>();
    blockAssembly->loadFromFile(ROBOCRAFT_DATA_FOLDER "/block/dome.obj");
    renderBlocks(true);
    blockAssembly->friction_mu_ = 0.5;
    std::shared_ptr<search::PartGraph> part_graph = std::make_shared<search::PartGraph>(blockAssembly);
    std::shared_ptr<search::StateGraphHolding> stateGraph = std::make_shared<search::StateGraphHolding>(part_graph, 2);
        std::shared_ptr<search::SearchAlgorithmBeamSearch> search = std::make_shared<search::SearchAlgorithmBeamSearch>(stateGraph, 10);
    search->search(sequence);
    sequence_id = 0;
    // Add content to the default menu window
    polyscope::state::userCallback = [&]()
    {
        //mouseEventCallback();

        int slider_max = std::max(0, (int) sequence.steps.size() - 1);
        if(ImGui::SliderInt("sequence id", &sequence_id, 0, slider_max)) {
            renderSequence();
        }
    };

    polyscope::show();
}