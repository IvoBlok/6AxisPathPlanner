#include "renderer/core/RenderEngine.hpp"

#include <stdexcept>
#include <iostream>

int main() {

	try {
		RenderEngine renderer;
		renderer.initialize();

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