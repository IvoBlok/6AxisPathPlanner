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

		std::vector<core::Vector3<double>> testLine = {
			core::Vector3<double>{1.f, 1.f, 2.f}, 
			core::Vector3<double>{-1.f, 1.f, 2.f}, 
			core::Vector3<double>{-1.f, -1.f, 2.f}, 
			core::Vector3<double>{1.f, -1.f, 2.f}, 
			core::Vector3<double>{1.f, 1.f, 2.f}
		};
		LoadedLine& line = renderer.createLine(testLine);
		
		core::Plane<double> plane;
		std::vector<core::Polyline2D<double>> meshPlaneIntersect = meshIntersect::getMeshPlaneIntersection(plane, bikeShell.updateObjectShape());

		for(int i=0; i < meshPlaneIntersect.size(); i++) {
			renderer.createLine(meshPlaneIntersect[i], plane);
		}

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