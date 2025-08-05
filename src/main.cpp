#include "renderer/core/RenderEngine.hpp"

#include "renderer/gui/testing/meshIntersectGUITest.hpp"
#include "renderer/gui/testing/robotGUITest.hpp"
#include "renderer/gui/testing/toolPath2_5DGUITest.hpp"

#include "renderer/gui/pathPlannerGUI.hpp"

#include <stdexcept>
#include <iostream>

int main() {

	try {
		RenderEngine renderer;
		renderer.initialize();

		meshIntersect::MeshIntersectGUITest meshIntersectGuiTest{renderer};
		toolPath2_5D::ToolPath2_5DGUITest toolPath2_5DGuiTest{renderer};
		kinematics::RobotGUITest robotGuiTest{renderer};
		PathPlannerGUI pathPlannerGUI{renderer};

		renderer.createDefaultCube("cube", Vector3f{0.f, 1.f, 1.f}, Vector3d{-0.6f, 0.f, 0.f}, Vector3d{.5f, .5f, .5f}, Vector3d{0.f, 0.f, 0.f}, 1.f);
		renderer.createDefaultCube("cube", Vector3f{1.f, 1.f, 0.f}, Vector3d{0.6f, 0.f, 0.f}, Vector3d{.5f, .5f, .5f}, Vector3d{0.f, 0.f, 0.f}, 0.5f);

		while (!renderer.shouldWindowClose()) {
			renderer.handleFrame();
		}
		renderer.cleanup();
	}
	catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}