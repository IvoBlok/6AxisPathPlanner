#include "renderer.hpp"
#include "meshIntersectGUI.hpp"
#include "toolPath2_5DGUI.hpp"

int main() {
	try {
		VulkanRenderEngine renderer;
		renderer.initialize();

		meshIntersect::MeshIntersectGUI meshIntersectGui;
		meshIntersectGui.registerWithRenderer(renderer);

		toolPath2_5D::ToolPath2_5DGUI toolPath2_5DGui;
		toolPath2_5DGui.registerWithRenderer(renderer);

		// form the actual rendering loop
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