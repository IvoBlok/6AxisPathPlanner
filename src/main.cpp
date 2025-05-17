#include <iostream>

#include "Renderer.hpp"

int main() {
	std::cout << "Hello world!\n";


	try {
		VulkanRenderEngine renderer;
		renderer.initialize();

		LoadedObject& bikeShell = renderer.createObject(
			"../../resources/assets/cube.obj",
			cavc::Vector3<double>{ 0.f, 0.f, 1.f },
			cavc::Vector3<double>{ 0.f, 5.f, 0.f },
			cavc::Vector3<double>{ 1.f, 1.f, 1.f },
			glm::mat4{ 1.f },
			1.f
		);
		
		// form the actual rendering loop
		while (!glfwWindowShouldClose(renderer.window)) {
			glfwPollEvents();
			
			// render the frame
			renderer.drawFrame();
		}

		// wait for the GPU to potentially finish the frame before terminating and cleaning up memory that might still be in use
		// vkDeviceWaitIdle(device);
				
		renderer.cleanup();
	}
	catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}


	return 0;
}