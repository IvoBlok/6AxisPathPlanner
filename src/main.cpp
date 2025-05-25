#include "renderer.hpp"
#include "meshIntersectGUI.hpp"

int main() {
	try {
		VulkanRenderEngine renderer;
		renderer.initialize();

		// register meshIntersect Module to be rendered
		MeshIntersectGUI meshIntersectGui{};
		meshIntersectGui.registerWithRenderer(renderer);

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