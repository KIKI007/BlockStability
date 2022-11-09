#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/file_dialog_open.h>
#include "block/BlockAssembly.h"

void drawBlock(igl::opengl::glfw::Viewer &viewer,
               std::shared_ptr<block::BlockAssembly> blockAssembly){
    viewer.data_list.clear();

    int nV = 0, nF = 0, nN = 0;
    for (int id = 0; id < blockAssembly->blocks_.size(); id++) {
        igl::opengl::ViewerData blockData = igl::opengl::ViewerData();
        auto block = blockAssembly->blocks_[id];
        nV += block->V_.rows();
        nF += block->F_.rows();
        nN += block->N_.rows();
    }

    Eigen::MatrixXd V(nV, 3);
    Eigen::MatrixXi F(nF, 3);
    Eigen::MatrixXd N(nN, 3);
    Eigen::MatrixXd C(nF, 3);

    int iV = 0; int iF = 0; int iN = 0;
    for (int id = 0; id < blockAssembly->blocks_.size(); id++)
    {
        auto block = blockAssembly->blocks_[id];
        V.block(iV, 0, block->V_.rows(), 3) = block->V_;
        F.block(iF, 0, block->F_.rows(), 3) = block->F_ + Eigen::MatrixXi::Ones(block->F_.rows(), 3) * iV;
        N.block(iN, 0, block->N_.rows(), 3) = block->N_;
        if(block->ground_){
            C.block(iF, 0, block->F_.rows(), 3) =  Eigen::MatrixXd::Ones(block->F_.rows(), 3) * 0.1;
        }
        else{
            C.block(iF, 0, block->F_.rows(), 3) =  Eigen::MatrixXd::Ones(block->F_.rows(), 3) * 0.8;
        }
        iV += block->V_.rows();
        iF += block->F_.rows();
        iN += block->N_.rows();
    }

    igl::opengl::ViewerData blockData = igl::opengl::ViewerData();
    blockData.id = viewer.data_list.size();
    blockData.set_mesh(V, F);
    blockData.set_colors(C);
    blockData.set_normals(N);
    blockData.face_based = true;
    viewer.data_list.push_back(blockData);
    viewer.selected_core_index = 0;
    viewer.selected_data_index = 0;
}

void drawContactNormal(igl::opengl::glfw::Viewer &viewer,
                       std::shared_ptr<block::BlockAssembly> blockAssembly){

    igl::opengl::ViewerData data = igl::opengl::ViewerData();
    std::vector<int> partIDs;
    for(int id = 0; id < blockAssembly->blocks_.size(); id++){
        partIDs.push_back(id);
    }
    auto graph = blockAssembly->computeContactGraph(partIDs);
    for(int id = 0; id < graph->contacts_.size(); id++)
    {
        Eigen::RowVector3d point = std::get<3>(graph->contacts_[id]);
        Eigen::RowVector3d normal = std::get<2>(graph->contacts_[id]);
        data.add_points(point, Eigen::RowVector3d(1, 0, 0));
        data.add_edges(point, point + normal * 0.1, Eigen::RowVector3d(0, 1, 0));
    }
    data.id = viewer.data_list.size();
    viewer.data_list.push_back(data);
}

void drawPartIndex(igl::opengl::glfw::Viewer &viewer,
                   std::shared_ptr<block::BlockAssembly> blockAssembly){
    igl::opengl::ViewerData data = igl::opengl::ViewerData();
    for(int id = 0; id < blockAssembly->blocks_.size(); id++){
        auto block = blockAssembly->blocks_[id];
        Eigen::RowVector3d ct = block->centroid();
        data.add_points(ct, Eigen::RowVector3d(0, 1, 0));
        data.add_label(ct.transpose(), std::to_string(id));
    }

    for(int id = 0; id < blockAssembly->support_forces_.size(); id++){
        auto block = blockAssembly->blocks_[id];
        Eigen::RowVector3d ct = block->centroid();
        Eigen::RowVector3d force = blockAssembly->support_forces_[id];
        data.add_edges(ct, ct + 10 * force, Eigen::RowVector3d(0, 0, 1));
    }

    data.id = viewer.data_list.size();
    viewer.data_list.push_back(data);
}

void drawContact(igl::opengl::glfw::Viewer &viewer,
                 std::shared_ptr<block::BlockAssembly> blockAssembly)
{

    std::vector<block::Contact> contacts;
    std::vector<int> partIDs;
    for(int id = 0; id < blockAssembly->blocks_.size(); id++)
    {
        partIDs.push_back(id);
    }
    blockAssembly->computeContacts(partIDs, contacts);

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
    igl::opengl::ViewerData contactData = igl::opengl::ViewerData();

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
    contactData.set_mesh(V, F);
    contactData.id = viewer.data_list.size();
    viewer.data_list.push_back(contactData);
    viewer.selected_core_index = 0;
    viewer.selected_data_index = 0;
}

void drawContent(igl::opengl::glfw::Viewer &viewer,
                 std::shared_ptr<block::BlockAssembly> blockAssembly,
                 bool show_contact,
                 bool show_contact_normal,
                 bool show_index) {
    viewer.data_list.clear();
    if(show_contact) drawContact(viewer, blockAssembly);
    if(show_contact_normal) drawContactNormal(viewer, blockAssembly);
    if(show_index) drawPartIndex(viewer, blockAssembly);
}

int main()
{
    std::shared_ptr<block::BlockAssembly> blockAssembly;

    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);


    static bool show_contact = false;
    static bool show_contact_normal = false;
    static bool show_index = false;
    static float fr_coeff = 0;
    // Add content to the default menu window
    menu.callback_draw_viewer_menu = [&](){
        // Draw parent menu content
        //menu.draw_viewer_menu();

        if (ImGui::CollapsingHeader("I/O", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if(ImGui::Button("Open File")){
                std::string obj_file = igl::file_dialog_open();
                blockAssembly = std::make_shared<block::BlockAssembly>();
                blockAssembly->loadFromFile(obj_file);
                drawBlock(viewer, blockAssembly);
            }

            if(ImGui::SliderFloat("friction coeff", &fr_coeff, 0, 1)){

            }

            if(ImGui::Button("Check Stability"))
            {
                if(blockAssembly){
                    std::vector<int> partIDs;
                    blockAssembly->friction_mu_ = fr_coeff;
                    for(int id = 0; id < blockAssembly->blocks_.size(); id++) {
                        partIDs.push_back(id);
                    }
                    if(blockAssembly->checkStability(partIDs))
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
            if(ImGui::Checkbox("contact", &show_contact))
            {
                drawContent(viewer, blockAssembly, show_contact, show_contact_normal, show_index);
            }

            if(ImGui::Checkbox("contact normal", &show_contact_normal))
            {
                drawContent(viewer, blockAssembly, show_contact, show_contact_normal, show_index);
            }

            if(ImGui::Checkbox("show index", &show_index)){
                drawContent(viewer, blockAssembly, show_contact, show_contact_normal, show_index);
            }
        }

        if(ImGui::CollapsingHeader("Boundary", ImGuiTreeNodeFlags_CollapsingHeader)){
            if(blockAssembly){
                for(int id = 0; id < blockAssembly->blocks_.size(); id++)
                {
                    std::string block_str = "block" + std::to_string(id);
                    if(ImGui::Checkbox(block_str.c_str(), &blockAssembly->blocks_[id]->ground_)){
                        drawBlock(viewer, blockAssembly);
                    }
                }
            }
        }
    };

    viewer.launch();
}