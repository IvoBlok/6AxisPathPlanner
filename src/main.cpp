#include <iostream>

#include "Renderer.hpp"

int main() {
	try {
		VulkanRenderEngine renderer;
		renderer.initialize();

		LoadedObject& bikeShell = renderer.createObject(
			"../../resources/assets/cube.obj",
			core::Vector3<double>{ 0.f, 0.f, 1.f },
			core::Vector3<double>{ 0.f, 5.f, 0.f },
			core::Vector3<double>{ 1.f, 1.f, 1.f },
			glm::mat4{ 1.f },
			1.f
		);
		
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