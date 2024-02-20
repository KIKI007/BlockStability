#include <imgui.h>
#include "render/file_dialog_open.h"
#include "render/AssemblyRender.h"

std::shared_ptr<rigid_block::Assembly> assembly = nullptr;
std::shared_ptr<render::AssemblyRender> assemblyRender = nullptr;

int main()
{
    polyscope::init();
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::automaticallyComputeSceneExtents = false;
    polyscope::state::boundingBox =
    std::tuple<glm::vec3, glm::vec3>{ {-1., -1., 0.}, {1., 1., 1.} };
    // Add content to the default menu window
    polyscope::state::userCallback = [&]()
    {

        if(assemblyRender) {
            assemblyRender->mouseEvent();
        }

        if (ImGui::CollapsingHeader("UI", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Open File")) {
                std::string obj_file = file_dialog_open();
                assembly = std::make_shared<rigid_block::Assembly>();
                assembly->loadFromFile(obj_file);
                assemblyRender.reset();
                assemblyRender = std::make_shared<render::AssemblyRender>("Assembly", assembly);
            }

            ImGui::SameLine();

            if (ImGui::Button("Check"))
            {
                assemblyRender->compute();
            }

            if(assemblyRender) {
                assemblyRender->gui();
            }
        }
    };

    // Give control to the polyscope gui
    polyscope::show();
}
