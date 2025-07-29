#ifndef RENDER_ENGINE_HPP
#define RENDER_ENGINE_HPP

#include "RenderCoreTypes.hpp"

#include <memory>
#include <map>
#include <unordered_map>
#include <vector>
#include <functional>

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

    // TODO implement these again
    /*
    std::shared_ptr<renderer::Object> createObject(...);
    std::shared_ptr<renderer::Curve> createCurve(...);
    void deleteObject(...);
    void deleteCurve(...);

    std::chrono::microseconds getDeltaTime();
    */

private:
    std::list<std::shared_ptr<renderer::Curve>> curves;
    std::list<std::shared_ptr<renderer::Object>> objects;
    
	std::vector<std::function<void(RenderEngine&)>> guiCallbacks;

    struct VulkanInternals;
    std::unique_ptr<VulkanInternals> vulkanInternals;

    void recordGUI();
};

#endif