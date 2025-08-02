#ifndef TOOLPATH_GUI_BASE_HPP
#define TOOLPATH_GUI_BASE_HPP

#include "renderer/core/RenderEngine.hpp"
#include "renderer/geometries/RendererObject.hpp"
#include "renderer/geometries/RendererCurve.hpp"

#include <functional>

template <typename T>
inline void handleDropdown(std::string ID, std::string title, const std::vector<std::unique_ptr<T>>& objects, int& selectionIndex, std::string defaultOptionText = "...") {
    ImGui::PushID(ID.c_str());

    if (!title.empty())
        ImGui::Text(title.c_str());

    std::string preview = defaultOptionText;
    if (selectionIndex >= 0 && selectionIndex < (int)objects.size()) {
        preview = objects[selectionIndex]->getName();
    }

    std::string comboLabel = "##" + ID + "_combo";
    if (ImGui::BeginCombo(comboLabel.c_str(), preview.c_str())) {
        for (int i = 0; i < (int)objects.size(); ++i) {
            ImGui::PushID(i);
            const bool isSelected = (selectionIndex == i);
            if (ImGui::Selectable(objects[i]->getName().c_str(), isSelected)) {
                selectionIndex = i; // Update the selection selectionIndex
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::PopID();
}

template <typename T, typename Predicate>
inline void handleDropdown(std::string ID, std::string title, const std::list<std::shared_ptr<T>>& objects, std::shared_ptr<T>& selection, Predicate useObject = [](const std::shared_ptr<T>&) { return true; }, std::string defaultOptionText = "...") {

    if (selection && !selection->isAlive())
        selection = nullptr;

    ImGui::PushID(ID.c_str());

    if (!title.empty())
        ImGui::Text(title.c_str());

    const char* preview = (selection) ? selection->getName().c_str() : defaultOptionText.c_str();
    
    std::string comboLabel = "##" + ID + "_combo";
    if (ImGui::BeginCombo(comboLabel.c_str(), preview)) {

        // retrieve the objects that comply with the input function
        std::vector<std::shared_ptr<T>> validObjects;
        validObjects.reserve(objects.size());

        for (const auto& object : objects)
            if (useObject(object))
                validObjects.emplace_back(object);

        for (int i = 0; i < (int)validObjects.size(); ++i) {
            ImGui::PushID(i);
            const bool isSelected = (selection.get() == validObjects[i].get());
            if (ImGui::Selectable(validObjects[i]->getName().c_str(), isSelected)) {
                selection = validObjects[i];
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::PopID();
}

class ToolPathGUIBase {
public:
    explicit ToolPathGUIBase(RenderEngine& renderer) : renderer(renderer) {}
    virtual ~ToolPathGUIBase() {}
    virtual void draw(bool isOpen) = 0;
    virtual std::string getName() const = 0;
    virtual std::unique_ptr<ToolPathGUIBase> clone() const = 0;

protected:
    RenderEngine& renderer;
};  

#endif