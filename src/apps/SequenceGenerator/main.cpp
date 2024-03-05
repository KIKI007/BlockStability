#include "rigid_block/Assembly.h"
#include "render/AssemblyRender.h"
#include "search/SearchAlgorithmBeamSearch.h"
#include "render/file_dialog_open.h"
#include "render/RobotRender.h"
#include "scene/Scene.h"

std::shared_ptr<rigid_block::Assembly> assembly = nullptr;
std::shared_ptr<render::AssemblyRender> assemblyRender = nullptr;
std::vector<std::shared_ptr<render::RobotRender>> robots;
std::shared_ptr<scene::Scene> scene1;

int sequence_id = 0;
search::AssemblySequence sequence;

Eigen::MatrixXd toMatrixXd(const polyscope::render::ManagedBuffer<glm::vec3> &pts) {
    Eigen::MatrixXd V(pts.data.size(), 3); V.setZero();
    for(int id = 0; id < pts.data.size(); id++) {
        for(int jd = 0; jd < 3; jd++) {
            V(id, jd) = pts.data.at(id)[jd];
        }
    }
    return V;
}

void check_collision()
{
    if(!sequence.steps.empty()) {
        std::vector<int> subset_part_ids = assemblyRender->installed_part_ids_;
        std::vector<int> robot_ids = sequence.steps[sequence_id].robot_ids_;
        std::vector<Eigen::VectorXd> joint_angles = sequence.steps[sequence_id].robot_angles_;

        switch (scene1->checkCollision(subset_part_ids, robot_ids, joint_angles))
        {
            case scene::RobotRobot:
                polyscope::warning("Collision among Robots");
                break;
            case scene::RobotGround:
                polyscope::warning("Collision between Robot and Ground");
                break;
            case scene::RobotAssembly:
                polyscope::warning("Collision between Robot and Assembly");
                break;
            default:
                break;
        }
    }
}

void renderSequence()
{
    assemblyRender->installed_part_ids_.clear();

    for(int id = 0; id < sequence.steps.size() && id < sequence_id + 1; id++)
    {
        auto installPartIDs = sequence.steps[id].installPartIDs_;
        for(auto partID : installPartIDs) {
            assemblyRender->installed_part_ids_.push_back(partID);
        }
    }

    assemblyRender->held_part_ids_ = sequence.steps[sequence_id].holdPartIDs_;
    assemblyRender->compute();
    assemblyRender->update();

    for(auto &robot: robots) {
        robot->j_.setZero();
        robot->eelist_.clear();
        robot->update();
    }

    for(int id = 0; id < assemblyRender->held_part_ids_.size(); id++)
    {
        int partID = assemblyRender->held_part_ids_[id];
        int actorID = sequence.steps[sequence_id].robot_ids_[id];
        robots[actorID]->eelist_ = assembly->blocks_[partID]->eeAnchor();
        robots[actorID]->j_ = sequence.steps[sequence_id].robot_angles_[id].cast<float>();
        robots[actorID]->update();
    }
}

void computeSequence()
{
    assembly->friction_coeff_ = assemblyRender->friction_coeff_;
    std::shared_ptr<search::SearchGraph> graph = std::make_shared<search::SearchGraph>(scene1);
    std::shared_ptr<search::SearchAlgorithmBeamSearch> search = std::make_shared<search::SearchAlgorithmBeamSearch>(graph, 10);
    search->search(sequence);
}

int main() {
    polyscope::init();
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::Tile;
    polyscope::options::automaticallyComputeSceneExtents = false;
    polyscope::state::boundingBox =
    std::tuple<glm::vec3, glm::vec3>{ {-2., -2., 0.}, {2., 2., 2.} };

    auto robot = std::make_shared<render::RobotRender>("robot1", Eigen::Vector3d(-1.5, 0, 0));
    robots.push_back(robot);
    robot = std::make_shared<render::RobotRender>("robot2", Eigen::Vector3d(1.5, 0, 0));
    robots.push_back(robot);

    sequence_id = 0;
    // Add content to the default menu window
    polyscope::state::userCallback = [&]()
    {
        int slider_max = std::max(0, (int) sequence.steps.size() - 1);

        if(assemblyRender) {
            assemblyRender->mouseEvent();
        }

        if(ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_RightArrow))) {
            if(slider_max != 0) {
                sequence_id = (sequence_id + 1) % slider_max;
                renderSequence();
            }
        }


        if(ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_LeftArrow))) {
            if(slider_max != 0) {
                sequence_id = (sequence_id - 1 + slider_max) % slider_max;
                renderSequence();
            }
        }

        if (ImGui::Button("Open File"))
        {
            std::string obj_file = file_dialog_open();
            assembly = std::make_shared<rigid_block::Assembly>();
            assembly->loadFromFile(obj_file);
            assemblyRender.reset();
            assemblyRender = std::make_shared<render::AssemblyRender>("", assembly);
            sequence_id = 0;

            std::vector<std::shared_ptr<robot::Robot>> rs;
            for(auto &r: robots) {
                rs.push_back(r->robot_);
            }
            scene1 = std::make_shared<scene::Scene>(assembly, rs);

            computeSequence();

        }

        if(assemblyRender)
        {
            ImGui::SameLine();

            if (ImGui::Button("Compute")) {
                computeSequence();
                sequence_id = 0;
            }

            ImGui::SameLine();

            if(ImGui::Button("Check")) {
                check_collision();
            }

            if(ImGui::SliderInt("sequence id", &sequence_id, 0, slider_max)) {
                renderSequence();
            }


            assemblyRender->gui();
        }

        for(auto robot : robots) {
            robot->gui();
        }
    };

    polyscope::show();
}