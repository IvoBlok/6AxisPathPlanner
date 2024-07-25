
#include "Renderer.h"

#include "MeshIntersect.h"
#include "cavc/polylineoffset.hpp"
#include "ToolpathGenerator.h"
#include "PolylineSimplifier.h"
#include "PathExporter.h"

VulkanRenderEngine renderer;

void generate2_5DWallPath(MillingPass2_5DInfo& millingInfo) {
	// calculate the toolpaths
	cavc::Polyline3D<double> toolPath = ToolpathGenerator::generate2_5DOutsideToolPath(millingInfo, cavc::Vector3<double>{ 0.f, 0.f, 1.f}, millingInfo.planeStartingHeight, millingInfo.planeEndingHeight);
	//cavc::Polyline3D<double> toolPath = ToolpathGenerator::generate2_5DOutsideToolPath(millingInfo, cavc::Vector3<double>{ 0.f, 0.f, 1.f}, .4f, .39f);

	// remove all arcs by converting them to series of lines, since the robot arm can not move in arcs
	cavc::Polyline3D<double> lineBasedToolPath = PathExporter::reducePathComplexity(toolPath);
	PathExporter::export3DPath(toolPath, millingInfo.filename);

	// to be properly displayed, the toolpath points need to be relative to the world zero, not the stock real zero
	cavc::Vector3<double> stockRealZeroPoint = millingInfo.stockInfo.zeroPoint;
	stockRealZeroPoint.z() += 0.001f * millingInfo.stockInfo.height;
	for (auto& move : lineBasedToolPath.vertexes())
	{
		move.point += stockRealZeroPoint;
		move.plane.origin += stockRealZeroPoint;
	}

	renderer.loadLine(lineBasedToolPath, 1.f, cavc::Vector3<double>{ 0.f, 0.f, 1.f});
	renderer.addGizmo(stockRealZeroPoint, 0.1f);
}

int main() {

	try {
		// initialize all parts of the renderer
		renderer.initialize();
		renderer.addGizmo();

		// load object to be milled to the renderer
		glm::mat4 rotationMatrix = glm::rotate(glm::mat4{ 1.f }, (float)(0.5f * PI), glm::vec3{ 0.f, 0.f, 1.f });
		rotationMatrix = glm::rotate(rotationMatrix, (float)(0.5f * PI), glm::vec3{ 1.f, 0.f, 0.f });
		LoadedObject& bikeShell = renderer.createObject(
			"models/RACING_VELO_G_BIRD.obj",
			cavc::Vector3<double>{ 0.3f, 0.3f, 0.3f },
			cavc::Vector3<double>{ 0.f, 0.f, 0.f },
			cavc::Vector3<double>{ 0.1f, 0.1f, 0.1f },
			rotationMatrix,
			1.f);

		// load the visualization planes and cube
		LoadedObject& safeTraversePlaneVisualization = renderer.createObject(
			"models/unitPlane.obj",
			cavc::Vector3<double>{ 1.f, 1.f, 0.f },
			cavc::Vector3<double>{ 0.f, 0.f, 0.f },
			cavc::Vector3<double>{ 1.f, 1.f, 1.f },
			glm::mat4{ 1.f },
			0.4f);

		LoadedObject& finalCuttingPlaneVisualization = renderer.createObject(
			"models/unitPlane.obj",
			cavc::Vector3<double>{ 0.f, 0.f, 1.f },
			cavc::Vector3<double>{ 0.f, 0.f, 0.f },
			cavc::Vector3<double>{ 1.f, 1.f, 1.f },
			glm::mat4{ 1.f },
			0.4f);

		LoadedObject& startCuttingPlaneVisualization = renderer.createObject(
			"models/unitPlane.obj",
			cavc::Vector3<double>{ 0.f, 0.2f, 1.f },
			cavc::Vector3<double>{ 0.f, 0.f, 0.f },
			cavc::Vector3<double>{ 1.f, 1.f, 1.f },
			glm::mat4{ 1.f },
			0.4f);

		LoadedObject& stockVisualization = renderer.createObject(
			"models/unitCube.obj",
			cavc::Vector3<double>{ 0.f, 0.f, 0.f },
			cavc::Vector3<double>{ 0.f, 0.f, 0.f },
			cavc::Vector3<double>{ 1.f, 1.f, 1.f },
			glm::mat4{ 1.f },
			0.2f);

		// set general milling info struct variables
		MillingPass2_5DInfo millingInfo;

		// load the object data into a more usable format
		bikeShell.model.loadModelDataIntoDesiredShapeContainer(millingInfo.desiredShape, bikeShell.getTransformationMatrix());

		// set temp robotInfo settings
		millingInfo.robotInfo.homePoint = cavc::Vector3<double>{ 0.f, 0.f, 0.f };
		millingInfo.robotInfo.zeroPoint = cavc::Vector3<double>{ 0.f, 0.f, 0.f };

		// set temp toolInfo settings
		millingInfo.toolInfo.fluteCount = 2;
		millingInfo.toolInfo.mainToolRadius = 10.f;
		millingInfo.toolInfo.toolCuttingHeight = 50.f;
		millingInfo.toolInfo.toolIsBallEnd = false;

		// set temp stockInfo settings
		millingInfo.stockInfo.zeroPoint = cavc::Vector3<double>{ -0.6f, -.25f, -.25f };
		millingInfo.stockInfo.height = 750.f;
		millingInfo.stockInfo.width = 1200.f;
		millingInfo.stockInfo.length = 500.f;

		// set general milling info
		millingInfo.depthOfCut = 40.f;
		millingInfo.stepOver = 18.f;
		millingInfo.safeTraverseHeight = 100.f;
		millingInfo.planeStartingHeight = 0.4f;
		millingInfo.planeEndingHeight = 0.39f;
		std::strcpy(millingInfo.filename, "testToolPath1.txt");

		// store all info that is to be set/modified in the GUI in the class
		SceneInfo sceneInfo{millingInfo};
		sceneInfo.generateToolPath = &generate2_5DWallPath;

		#pragma region rendering loop and cleanup
		// form the actual rendering loop
		while (!glfwWindowShouldClose(renderer.window)) {
			glfwPollEvents();

			// update visualization objects (planes + cube)
			// STOCK
			cavc::Vector3<double> stockSize{ millingInfo.stockInfo.width * 0.001f, millingInfo.stockInfo.length * 0.001f, millingInfo.stockInfo.height * 0.001f};
			cavc::Vector3<double> stockCenter = millingInfo.stockInfo.zeroPoint + (double)0.5f * stockSize;
			stockVisualization.position = glm::vec3{ stockCenter.x(), stockCenter.y(), stockCenter.z() };
			stockVisualization.scale = glm::vec3{ stockSize.x(), stockSize.y(), stockSize.z() };

			// SAFE TRAVEL PLANE
			safeTraversePlaneVisualization.position = glm::vec3{ stockCenter.x(), stockCenter.y(), millingInfo.stockInfo.zeroPoint.z() + millingInfo.planeStartingHeight + millingInfo.safeTraverseHeight * 0.001f };
			safeTraversePlaneVisualization.scale = 1.1f * glm::vec3{ stockSize.x(), stockSize.y(), stockSize.z() };

			// START MILLING PLANE
			startCuttingPlaneVisualization.position = glm::vec3{ stockCenter.x(), stockCenter.y(), millingInfo.stockInfo.zeroPoint.z() + millingInfo.planeStartingHeight };
			startCuttingPlaneVisualization.scale = 1.1f * glm::vec3{ stockSize.x(), stockSize.y(), stockSize.z() };

			// END MILLING PLANE
			finalCuttingPlaneVisualization.position = glm::vec3{ stockCenter.x(), stockCenter.y(), millingInfo.stockInfo.zeroPoint.z() + millingInfo.planeEndingHeight };
			finalCuttingPlaneVisualization.scale = 1.1f * glm::vec3{ stockSize.x(), stockSize.y(), stockSize.z() };

			renderer.drawFrame(sceneInfo);
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