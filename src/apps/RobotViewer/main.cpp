#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "robot/Robot.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "render/RobotRender.h"

std::shared_ptr<render::RobotRender> robot_render;

Eigen::VectorXf angle;
Eigen::Vector3f EExyz;
Eigen::Vector3f EErpy;
int iksol = 0;
double ee_arrow_length = 0.1;
double ee_point_radius = 0.02;
double ee_arrow_radius = 0.01;

int main() {

    polyscope::init();
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::automaticallyComputeSceneExtents = false;
    polyscope::state::boundingBox =
    std::tuple<glm::vec3, glm::vec3>{ {-1., -1., 0.}, {1., 1., 1.} };

    robot_render = std::make_shared<render::RobotRender>("robot1", Eigen::Vector3d(0, 0, 0));

    polyscope::state::userCallback = [&]()
    {
        if(robot_render) {
            robot_render->gui();
        }
    };

    polyscope::show();
}