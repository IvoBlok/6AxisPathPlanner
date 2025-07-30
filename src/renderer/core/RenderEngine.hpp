#ifndef RENDER_ENGINE_HPP
#define RENDER_ENGINE_HPP

#include "RenderCoreTypes.hpp"
#include "CustomEigen.hpp"

#include <memory>
#include <map>
#include <unordered_map>
#include <vector>
#include <functional>
#include <string>
#include <chrono>

namespace renderer {
    class Curve;
    class Object;
}

class RenderEngine {
public:

    RenderEngine();
    ~RenderEngine();

    void initialize();
    void handleFrame();
    void cleanup();
    bool shouldWindowClose();

	void registerGuiModule(std::function<void(RenderEngine&)> callback);
    renderer::VulkanContext& getContext() const;
    
    std::shared_ptr<renderer::Object> createObject(
        const char* modelPath,
        const char* texturePath,
        std::string name,
        Vector3d basePosition,
        Vector3d baseScale,
        Vector3d baseRotation,
        float transparency
    );
    std::shared_ptr<renderer::Object> createObject(
        const char* modelPath,
        std::string name,
        Vector3f color,
        Vector3d basePosition,
        Vector3d baseScale,
        Vector3d baseRotation,
        float transparency
    );

    // TODO createCurve functions, createDefaultCube, etc...
    std::shared_ptr<renderer::Object> createDefaultCube(
        std::string name = "cube",
        Vector3f color = Vector3d::Zero(),
        Vector3d basePosition = Vector3d::Zero(),
        Vector3d baseScale = Vector3d::Ones(),
        Vector3d baseRotation = Vector3d::Zero(),
        float transparency = 1.f
    );

    std::shared_ptr<renderer::Object> createDefaultPlane(
        std::string name = "plane",
        Vector3f color = Vector3d::Zero(),
        Vector3d basePosition = Vector3d::Zero(),
        Vector3d baseScale = Vector3d::Ones(),
        Vector3d baseRotation = Vector3d::Zero(),
        float transparency = 1.f
    );

    void deleteObject(std::shared_ptr<renderer::Object>& object);
    void deleteObject(renderer::Object* object);
    void deleteCurve(std::shared_ptr<renderer::Curve>& curve);
    void deleteCurve(renderer::Curve* curve);

    std::chrono::microseconds getDeltaTime();

private:
    std::list<std::shared_ptr<renderer::Curve>> curves;
    std::list<std::shared_ptr<renderer::Object>> objects;
    
	std::vector<std::function<void(RenderEngine&)>> guiCallbacks;

    struct VulkanInternals;
    std::unique_ptr<VulkanInternals> vulkanInternals;

    void recordGUI();
};

#endif