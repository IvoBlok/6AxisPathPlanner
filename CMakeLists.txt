cmake_minimum_required(VERSION 3.15)
project(RobotPathPlanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
# ============================================

# Vulkan
find_package(Vulkan REQUIRED)

# GLFW
include(FetchContent)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG latest
)
FetchContent_MakeAvailable(glfw)

# GLM
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
)
FetchContent_MakeAvailable(glm)

# ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG docking
)
FetchContent_MakeAvailable(imgui)

# ImPlot
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG master
)
FetchContent_MakeAvailable(implot)

set(GLM_TEST_ENABLE OFF CACHE BOOL "Disable GLM tests")

# stb_image
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG master  
)
FetchContent_MakeAvailable(stb)

# tiny_obj_loader
FetchContent_Declare(
    tinyobjloader
    GIT_REPOSITORY https://github.com/tinyobjloader/tinyobjloader.git
    GIT_TAG release
)
FetchContent_MakeAvailable(tinyobjloader)

# nfd (native file dialog)
FetchContent_Declare(
    nfd
    GIT_REPOSITORY https://github.com/mlabbe/nativefiledialog.git
    GIT_TAG master
)
FetchContent_MakeAvailable(nfd)

# Executable
# ============================================

# enable Debugging
set(CMAKE_BUILD_TYPE Debug)

add_executable(robotPathPlanner 
    src/main.cpp
    src/renderer.cpp
    src/meshIntersect.cpp
    src/toolPath2_5D.cpp
    external/stb_image.cpp
    external/tiny_obj_loader.cpp
)

target_compile_definitions(robotPathPlanner PRIVATE 
    GLM_ENABLE_EXPERIMENTAL 
)

# Copy shaders to build folder
# ============================================

add_custom_command(
    TARGET robotPathPlanner PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/shaders
        ${CMAKE_CURRENT_BINARY_DIR}/shaders
    COMMENT "Copying shaders to build directory"
)

if(CMAKE_BUILD_TYPE)
    # Single-config generators (Make, Ninja)
    set(SHADER_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/shaders)
else()
    # Multi-config generators (Visual Studio)
    set(SHADER_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/Debug/shaders)
endif()

# 2. Copy shaders to the correct output directory
file(MAKE_DIRECTORY ${SHADER_OUTPUT_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/resources/shaders/
      DESTINATION ${SHADER_OUTPUT_DIR})

# Include Directories
# ============================================
target_include_directories(robotPathPlanner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${Vulkan_INCLUDE_DIRS}
    ${glm_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${implot_SOURCE_DIR}        
    external/imgui
    ${stb_SOURCE_DIR}
    ${tinyobjloader_SOURCE_DIR}
    ${nfd_SOURCE_DIR}/src/include
)

# Link Libraries
# ============================================
target_link_libraries(robotPathPlanner PRIVATE 
    glfw
    ${Vulkan_LIBRARIES}
)

# Add Imgui Sources
# ============================================
target_sources(robotPathPlanner PRIVATE
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
)

# Add ImPlot Sources
# ============================================
target_sources(robotPathPlanner PRIVATE
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
)

# Add NFD Sources
# ============================================
if(WIN32) # Windows
    target_sources(robotPathPlanner PRIVATE
        ${nfd_SOURCE_DIR}/src/nfd_common.c
        ${nfd_SOURCE_DIR}/src/nfd_win.cpp
    )
else()  # Linux/Unix
    target_sources(robotPathPlanner PRIVATE
        ${nfd_SOURCE_DIR}/src/nfd_common.c
        ${nfd_SOURCE_DIR}/src/nfd_gtk.c
    )
    # For Linux systems, you'll need GTK3 development packages
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
    target_include_directories(robotPathPlanner PRIVATE ${GTK3_INCLUDE_DIRS})
    target_link_libraries(robotPathPlanner PRIVATE ${GTK3_LIBRARIES})
endif()