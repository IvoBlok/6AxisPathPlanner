#include "renderer.hpp"
#include "gui/testing/meshIntersectGUI.hpp"
#include "gui/testing/toolPath2_5DGUI.hpp"
#include "gui/testing/robotGUI.hpp"

#include "gui/pathPlannerGUI.hpp"

int main() {

	try {
		VulkanRenderEngine renderer;
		renderer.initialize();

		meshIntersect::MeshIntersectGUI meshIntersectGui{renderer};
		toolPath2_5D::ToolPath2_5DGUI toolPath2_5DGui{renderer};
		kinematics::RobotGUI robotGui{renderer};
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