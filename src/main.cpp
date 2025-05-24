#include <iostream>

#include "renderer.hpp"
#include "meshIntersect.hpp"

int main() {
	try {
		VulkanRenderEngine renderer;
		renderer.initialize();

		LoadedObject& bikeShell = renderer.createObject(
			"../../resources/assets/cube.obj",
			core::Vector3<double>{ 0.f, 0.f, 1.f }, // color
			core::Vector3<double>{ 0.f, 5.f, 0.f }  // position
		);
		core::Plane<double> plane;
		std::vector<core::Polyline2D<double>> meshPlaneIntersect = meshIntersect::getMeshPlaneIntersection(plane, bikeShell.updateObjectShape());
		
		LoadedLine& line = renderer.createLine(meshPlaneIntersect[0], plane);
		
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