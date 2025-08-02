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