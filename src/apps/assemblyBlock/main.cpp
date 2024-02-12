#include "RigidBlock/Assembly.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include <imgui.h>
#include "file_dialog_open.h"

std::vector<polyscope::SurfaceMesh *> block_render_objs;
polyscope::SurfaceMesh* contact_render_obj;

void drawBlock(std::shared_ptr<rigid_block::Assembly> blockAssembly, bool visible)
{
    block_render_objs.clear();
    for (int id = 0; id < blockAssembly->blocks_.size(); id++)
    {
        std::string name = "block" + std::to_string(id);
        auto block = blockAssembly->blocks_[id];
        auto block_obj = polyscope::registerSurfaceMesh(name,  block->V_,  block->F_);
        Eigen::Vector3d color = block->color();
        std::cout << block->ground_ << std::endl;
        block_obj->setSurfaceColor({color[0], color[1], color[2]});
        //block_obj->setEdgeWidth(1.0);
        block_obj->setEnabled(visible);
        block_render_objs.push_back(block_obj);
    }
}

void drawContact(std::shared_ptr<rigid_block::Assembly> blockAssembly, bool visible)
{
    std::vector<rigid_block::Contact> contacts;
    std::vector<int> partIDs;
    for(int id = 0; id < blockAssembly->blocks_.size(); id++)
    {
        partIDs.push_back(id);
    }
    contacts = blockAssembly->computeContacts(partIDs);

    int nV = 0;
    int nF = 0;
    for(int id = 0; id < contacts.size(); id++) {
        auto contact = contacts[id];
        nV += contact.points.size();
        nF += contact.points.size() - 2;
    }

    Eigen::MatrixXd V(nV, 3);
    Eigen::MatrixXi F(nF, 3);

    int iV = 0;
    int iF = 0;

    for(int id = 0; id < contacts.size(); id++)
    {
        auto contact = contacts[id];
        int nV = contact.points.size();
        for(int jd = 0; jd < nV; jd++){
            V.row(jd + iV) = contact.points[jd];
        }
        for (int jd = 2; jd < nV; jd++)
        {
            F(jd - 2 + iF, 0) = iV;
            F(jd - 2 + iF, 1)  = iV + jd - 1;
            F(jd - 2 + iF, 2) = iV + jd;
        }
        iV += nV;
        iF += nV - 2;
    }

    contact_render_obj = polyscope::registerSurfaceMesh("contact", V, F);
    contact_render_obj->setEnabled(visible);
}

// void drawContent(igl::opengl::glfw::Viewer &viewer,
//                  std::shared_ptr<rigid_block::Assembly> blockAssembly,
//                  bool show_contact,
//                  bool show_contact_normal,
//                  bool show_index) {
//     viewer.data_list.clear();
//     if(show_contact) drawContact(viewer, blockAssembly);
//     if(show_contact_normal) drawContactNormal(viewer, blockAssembly);
//     if(show_index) drawPartIndex(viewer, blockAssembly);
// }

int main()
{
    std::shared_ptr<rigid_block::Assembly> blockAssembly;

    static bool show_assembly = true;
    static bool show_assembly_wireframe = true;

    static bool show_contact = false;
    static bool show_contact_normal = false;
    static bool show_index = false;
    static float fr_coeff = 0;

    polyscope::init();

    // Add content to the default menu window
     polyscope::state::userCallback= [&]()
    {

        if (ImGui::CollapsingHeader("I/O", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if(ImGui::Button("Open File")){
                std::string obj_file = file_dialog_open();
                blockAssembly = std::make_shared<rigid_block::Assembly>();
                blockAssembly->loadFromFile(obj_file);
                drawBlock(blockAssembly, show_assembly);
                drawContact(blockAssembly, show_contact);
            }

            if(ImGui::SliderFloat("friction coeff", &fr_coeff, 0, 1)){

            }

            if(ImGui::Button("Check Stability"))
            {
                if(blockAssembly)
                {
                    std::vector<int> partIDs;
                    blockAssembly->friction_mu_ = fr_coeff;
                    auto analyzer = blockAssembly->createAnalyzer();
                    Eigen::VectorXd forces;
                    if(analyzer->checkStability(forces))
                    {
                        std::cout << "Stable" << std::endl;
                    }
                    else{
                        std::cout << "UnStable" << std::endl;
                    }
                }
            }
        }

        if(ImGui::CollapsingHeader("Rendering", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if(ImGui::Checkbox("assembly", &show_assembly))
            {
                for(auto obj : block_render_objs)
                    obj->setEnabled(show_assembly);
            }


            if(ImGui::Checkbox("contact", &show_contact))
            {
                contact_render_obj->setEnabled(show_contact);
            }

        //     if(ImGui::Checkbox("contact normal", &show_contact_normal))
        //     {
        //         drawContent(viewer, blockAssembly, show_contact, show_contact_normal, show_index);
        //     }
        //
        //     if(ImGui::Checkbox("show index", &show_index)){
        //         drawContent(viewer, blockAssembly, show_contact, show_contact_normal, show_index);
        //     }
        }
    };

    // Give control to the polyscope gui
    polyscope::show();
}