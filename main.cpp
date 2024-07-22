
#include "Renderer.h"

#include "MeshIntersect.h"
#include "cavc/polylineoffset.hpp"
#include "ToolpathGenerator.h"
#include "PolylineSimplifier.h"
#include "PathExporter.h"


int main() {
	VulkanRenderEngine renderer;

	try {
		// initialize all parts of the renderer
		renderer.initialize();

		// create visualizer in the renderer for the zero point and cardinal directions
		renderer.addGizmo(cavc::Vector3<double>{ 0.f, 0.f, 0.f }, 1.f);

		// load object to be milled to the renderer
		glm::mat4 rotationMatrix = glm::rotate(glm::mat4{ 1.f }, (float)(0.5f * PI), glm::vec3{ 0.f, 0.f, 1.f });
		rotationMatrix = glm::rotate(rotationMatrix, (float)(0.5f * PI), glm::vec3{ 1.f, 0.f, 0.f });
		LoadedObject& bikeShell = renderer.createObject("models/RACING_VELO_G_BIRD.obj", "models/RACING_VELO_G_BIRD.png", cavc::Vector3<double>{ 0.f, 0.f, 0.f }, cavc::Vector3<double>{ 0.1f, 0.1f, 0.1f }, rotationMatrix, 0.4f);

		// load the object data into a more usable format
		DesiredShape desiredShape;
		bikeShell.model.loadModelDataIntoDesiredShapeContainer(desiredShape, bikeShell.getTransformationMatrix());

		// set temp robotInfo settings
		RobotInfo robotInfo;
		robotInfo.homePoint = cavc::Vector3<double>{ 0.f, 0.f, 0.f };
		robotInfo.zeroPoint = cavc::Vector3<double>{ 0.f, 0.f, 0.f };

		// set temp toolInfo settings
		ToolInfo toolInfo;
		toolInfo.fluteCount = 2;
		toolInfo.mainToolRadius = 10.f;
		toolInfo.toolCuttingHeight = 50.f;
		toolInfo.toolIsBallEnd = false;

		// set temp stockInfo settings
		StockInfo stockInfo;
		stockInfo.zeroPoint = cavc::Vector3<double>{ -0.6f, -.25f, -.25f };
		stockInfo.height = 750.f;
		stockInfo.width = 1200.f;
		stockInfo.length = 500.f;

		// set general milling info struct variables
		MillingPass2_5DInfo millingInfo;
		millingInfo.desiredShape = desiredShape;
		millingInfo.robotInfo = robotInfo;
		millingInfo.toolInfo = toolInfo;
		millingInfo.stockInfo = stockInfo;

		millingInfo.depthOfCut = 40.f;
		millingInfo.stepOver = 18.f;
		millingInfo.safeTraverseHeight = 100.f;

		// calculate the toolpaths
		cavc::Polyline3D<double> toolPath = ToolpathGenerator::generate2_5DOutsideToolPath(millingInfo, cavc::Vector3<double>{ 0.f, 0.f, 1.f}, .4f, .39f);

		// remove all arcs by converting them to series of lines, since the robot arm can not move in arcs
		cavc::Polyline3D<double> lineBasedToolPath = PathExporter::reducePathComplexity(toolPath);
		PathExporter::export3DPath(toolPath, "testToolPath.txt");

		// to be properly displayed, the toolpath points need to be relative to the world zero, not the stock real zero
		cavc::Vector3<double> stockRealZeroPoint = millingInfo.stockInfo.zeroPoint;
		stockRealZeroPoint.z() += 0.001f * millingInfo.stockInfo.height;
		for (auto& move : lineBasedToolPath.vertexes())
		{
			move.point += stockRealZeroPoint;
			move.plane.origin += stockRealZeroPoint;
		}

		renderer.loadLine(lineBasedToolPath, 1.f, cavc::Vector3<double>{ 0.f, 0.f, 1.f});
		renderer.addGizmo(stockRealZeroPoint, 1.f);
		
		#pragma region rendering loop and cleanup
		// form the actual rendering loop
		while (!glfwWindowShouldClose(renderer.window)) {
			glfwPollEvents();
			renderer.drawFrame();
		}

		// wait for the GPU to potentially finish the frame before terminating and cleaning up memory that might still be in use
		vkDeviceWaitIdle(device);

		renderer.cleanup();
		#pragma endregion
	}
	catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}