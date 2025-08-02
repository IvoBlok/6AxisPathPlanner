#ifndef RENDER_ENGINE_HPP
#define RENDER_ENGINE_HPP

#include "RenderCoreTypes.hpp"
#include "CustomEigen.hpp"

#include "core/polyline.hpp"

#include "imconfig.h"
#include "imgui_internal.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan_but_better.hpp"	
#include "implot.h"

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
        std::string name = "object",
        Vector3d basePosition = Vector3d::Zero(),
        Vector3d baseScale = Vector3d::Ones(),
        Vector3d baseRotation = Vector3d::Zero(),
        float transparency = 1.f
    );
    std::shared_ptr<renderer::Object> createObject(
        const char* modelPath,
        std::string name = "object",
        Vector3f color = Vector3f::UnitX(),
        Vector3d basePosition = Vector3d::Zero(),
        Vector3d baseScale = Vector3d::Ones(),
        Vector3d baseRotation = Vector3d::Zero(),
        float transparency = 1.f
    );

    std::shared_ptr<renderer::Object> createDefaultCube(
        std::string name = "cube",
        Vector3f color = Vector3f::UnitX(),
        Vector3d basePosition = Vector3d::Zero(),
        Vector3d baseScale = Vector3d::Ones(),
        Vector3d baseRotation = Vector3d::Zero(),
        float transparency = 1.f
    );

    std::shared_ptr<renderer::Object> createDefaultPlane(
        std::string name = "plane",
        Vector3f color = Vector3f::UnitZ(),
        Vector3d basePosition = Vector3d::Zero(),
        Vector3d baseScale = Vector3d::Ones(),
        Vector3d baseRotation = Vector3d::Zero(),
        float transparency = 1.f
    );

    std::shared_ptr<renderer::Curve> createCurve(
        core::Polyline2_5D& polyline, 
        std::string name = "curve",
        Vector3f color = Vector3f::UnitY(),
        float transparency = 1.f
    );

    std::shared_ptr<renderer::Curve> createCurve(
        core::Polyline2D& polyline, 
        core::Plane& plane,
        std::string name = "curve",
        Vector3f color = Vector3f::UnitY(),
        float transparency = 1.f
    );

    void removeObject(std::shared_ptr<renderer::Object>& object);
    void removeObject(renderer::Object* object);
    void removeCurve(std::shared_ptr<renderer::Curve>& curve);
    void removeCurve(renderer::Curve* curve);

    std::chrono::microseconds getDeltaTime();

    std::list<std::shared_ptr<renderer::Curve>>& getCurves();
    std::list<std::shared_ptr<renderer::Object>>& getObjects();

private:
    std::list<std::shared_ptr<renderer::Curve>> curves;
    std::list<std::shared_ptr<renderer::Object>> objects;

    std::vector<std::shared_ptr<renderer::Object>> objectsToRemove;
    std::vector<std::shared_ptr<renderer::Curve>> curvesToRemove;

	std::vector<std::function<void(RenderEngine&)>> guiCallbacks;

    struct VulkanInternals;
    std::unique_ptr<VulkanInternals> vulkanInternals;

    void recordGUI();

    void processRemovals();
};

#endif