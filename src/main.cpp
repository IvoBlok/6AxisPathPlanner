#include <iostream>

#include "Renderer.hpp"

int main() {
	std::cout << "Hello world!\n";


	try {
		VulkanRenderEngine renderer;
		renderer.initialize();

		/*LoadedObject& bikeShell = renderer.createObject(
			"models/ligfietsbak-bottom.obj",
			cavc::Vector3<double>{ 0.3f, 0.3f, 0.3f });
		*/
		
		// form the actual rendering loop
		while (!glfwWindowShouldClose(renderer.window)) {
			glfwPollEvents();
			
			// render the frame
			renderer.drawFrame();
		}

		// wait for the GPU to potentially finish the frame before terminating and cleaning up memory that might still be in use
		vkDeviceWaitIdle(device);
				
		renderer.cleanup();
	}
	catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}


	return 0;
}