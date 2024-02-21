#include "rigid_block/Assembly.h"
#include "render/AssemblyRender.h"
#include "search/SearchAlgorithmBeamSearch.h"
#include "search/StateGraphHolding.h"
#include "render/file_dialog_open.h"
#include "render/RobotRender.h"

std::shared_ptr<rigid_block::Assembly> assembly = nullptr;
std::shared_ptr<render::AssemblyRender> assemblyRender = nullptr;
std::vector<std::shared_ptr<render::RobotRender>> robots_;

int sequence_id = 0;
search::AssemblySequence sequence;

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

    for(auto &robot: robots_) {
        robot->j_.setZero();
        robot->update();
    }

    for(int id = 0; id < (int) assemblyRender->held_part_ids_.size(); id++)
    {
        int partID = assemblyRender->held_part_ids_[id];
        int actorID = sequence.steps[sequence_id].actors_[id];
        util::Transform ee = assembly->blocks_[partID]->eeAnchor().front();
        robots_[actorID]->ee_rpy_ = Eigen::Vector3f(ee.rpy.x(), ee.rpy.y(), ee.rpy.z());
        robots_[actorID]->ee_xyz_ = Eigen::Vector3f(ee.xyz.x(), ee.xyz.y(), ee.xyz.z());
        robots_[actorID]->compute();
        robots_[actorID]->update();
    }

}

void computeSequence()
{
    assembly->friction_coeff_ = assemblyRender->friction_coeff_;
    std::shared_ptr<search::PartGraph> part_graph = std::make_shared<search::PartGraph>(assembly);
    std::shared_ptr<search::StateGraphHolding> stateGraph = std::make_shared<search::StateGraphHolding>(part_graph, 2);
    std::shared_ptr<search::SearchAlgorithmBeamSearch> search = std::make_shared<search::SearchAlgorithmBeamSearch>(stateGraph, 10);
    search->search(sequence);
}

int main() {
    polyscope::init();
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::automaticallyComputeSceneExtents = false;
    polyscope::state::boundingBox =
    std::tuple<glm::vec3, glm::vec3>{ {-1., -1., 0.}, {1., 1., 1.} };

    auto robot = std::make_shared<render::RobotRender>("robot1", Eigen::Vector3d(-1.5, 0, 0));
    robots_.push_back(robot);
    robot = std::make_shared<render::RobotRender>("robot2", Eigen::Vector3d(1.5, 0, 0));
    robots_.push_back(robot);

    sequence_id = 0;
    // Add content to the default menu window
    polyscope::state::userCallback = [&]()
    {
        if(assemblyRender) {
            assemblyRender->mouseEvent();
        }

        if (ImGui::Button("Open File"))
        {
            std::string obj_file = file_dialog_open();
            assembly = std::make_shared<rigid_block::Assembly>();
            assembly->loadFromFile(obj_file);
            assemblyRender.reset();
            assemblyRender = std::make_shared<render::AssemblyRender>("", assembly);
        }

        if(assemblyRender)
        {
            ImGui::SameLine();

            if (ImGui::Button("Compute")) {
                computeSequence();
                sequence_id = 0;
            }

            int slider_max = std::max(0, (int) sequence.steps.size() - 1);
            if(ImGui::SliderInt("sequence id", &sequence_id, 0, slider_max)) {
                renderSequence();
            }

            assemblyRender->gui();
        }

        for(auto robot : robots_) {
            robot->gui();
        }
    };

    polyscope::show();
}