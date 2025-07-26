#include "renderer.hpp"
#include "gui/testing/meshIntersectGUITest.hpp"
#include "gui/testing/toolPath2_5DGUITest.hpp"
#include "gui/testing/robotGUITest.hpp"

#include "gui/pathPlannerGUI.hpp"

int main() {

	try {
		VulkanRenderEngine renderer;
		renderer.initialize();

		meshIntersect::MeshIntersectGUITest meshIntersectGuiTest{renderer};
		toolPath2_5D::ToolPath2_5DGUITest toolPath2_5DGuiTest{renderer};
		kinematics::RobotGUITest robotGuiTest{renderer};
		PathPlannerGUI pathPlannerGUI{renderer};

		while (!glfwWindowShouldClose(renderer.window)) {
			renderer.drawFrame();
		}
		renderer.cleanup();
	}
	catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}