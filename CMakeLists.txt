cmake_minimum_required(VERSION 3.25)
project(RobotPathPlanner)

# Force Debug if no config specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    # Windows/MSVC settings
    add_compile_options(/W4 /WX-)
    add_link_options(/DEBUG:FULL)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    # Linux/GCC settings
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CONFIGURATION_TYPES)  # Multi-config
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release)
else()  # Single-config
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE})
endif()

# ============================================
# Dependencies
# ============================================

# Vulkan
find_package(Vulkan REQUIRED)

# Disable uninstall targets for all dependencies
set(ADD_UNINSTALL_TARGET OFF CACHE BOOL "Skip uninstall targets")
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "")
set(GLFW_BUILD_TESTS OFF CACHE BOOL "")
set(GLFW_BUILD_DOCS OFF CACHE BOOL "")
set(GLFW_INSTALL OFF CACHE BOOL "")

# GLFW
include(FetchContent)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)

# GLM
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
)

# ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG docking
)

# ImPlot
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG master
)

set(GLM_TEST_ENABLE OFF CACHE BOOL "Disable GLM tests")

# stb_image
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG master  
)

# tiny_obj_loader
FetchContent_Declare(
    tinyobjloader
    GIT_REPOSITORY https://github.com/tinyobjloader/tinyobjloader.git
    GIT_TAG release
)

# nfd (native file dialog)
FetchContent_Declare(
    nfd
    GIT_REPOSITORY https://github.com/mlabbe/nativefiledialog.git
    GIT_TAG master
)

FetchContent_MakeAvailable(
    glfw
    glm
    imgui
    implot
    stb
    tinyobjloader
    nfd
)


# Executable
# ============================================

add_executable(robotPathPlanner 
    src/renderer/core/RenderCoreTypes.cpp
    src/renderer/core/RenderEngine.cpp
    src/renderer/geometries/RendererGeometryBase.cpp
    src/renderer/geometries/RendererCurve.cpp
    src/renderer/geometries/RendererObject.cpp
    src/main.cpp
    src/meshIntersect.cpp
    src/toolPath2_5D.cpp
    src/robotKinematics.cpp
    external/stb_image.cpp
    external/tiny_obj_loader.cpp
)

target_compile_definitions(robotPathPlanner PRIVATE 
    GLM_ENABLE_EXPERIMENTAL 
)

# Include Directories
# ============================================
target_include_directories(robotPathPlanner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/external  
    ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/external/Eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${Vulkan_INCLUDE_DIRS}
    ${glm_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${implot_SOURCE_DIR}      
    ${stb_SOURCE_DIR}
    ${tinyobjloader_SOURCE_DIR}
    ${nfd_SOURCE_DIR}/src/include
)

# Copy shaders to build folder
# ============================================

find_program(GLSLC_EXECUTABLE glslc 
    HINTS "$ENV{VULKAN_SDK}/bin" "/usr/bin" "/usr/local/bin"
    REQUIRED
)

set(SHADER_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/shaders)
file(MAKE_DIRECTORY ${SHADER_OUTPUT_DIR})

function(compile_shader shader_path)
    get_filename_component(shader_name ${shader_path} NAME)
    set(output_path ${SHADER_OUTPUT_DIR}/${shader_name}.spv)
    
    # Generate unique target name
    string(MAKE_C_IDENTIFIER "shader_${shader_name}" target_name)
    
    add_custom_command(
        OUTPUT ${output_path}
        COMMAND ${GLSLC_EXECUTABLE} ${shader_path} -o ${output_path}
        DEPENDS ${shader_path}
        COMMENT "Compiling ${shader_name}"
        VERBATIM
    )
    
    add_custom_target(${target_name} ALL DEPENDS ${output_path})
    add_dependencies(robotPathPlanner ${target_name})
endfunction()

# Process all shaders
file(GLOB_RECURSE SHADER_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/shaders/*.vert"
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/shaders/*.frag"
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/shaders/*.comp"
)

foreach(shader ${SHADER_FILES})
    compile_shader(${shader})
endforeach()


# Link Libraries
# ============================================
target_link_libraries(robotPathPlanner PRIVATE 
    glfw
    ${Vulkan_LIBRARIES}
)

# Add Sources for the various libraries
# ============================================
target_sources(robotPathPlanner PRIVATE
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
)

target_sources(robotPathPlanner PRIVATE
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
)

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

# Set compiler flags for different build types
# ============================================
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_definitions(robotPathPlanner PRIVATE NDEBUG)
    if(MSVC)
        target_compile_options(robotPathPlanner PRIVATE /O2)
    else()
        target_compile_options(robotPathPlanner PRIVATE -O3)
    endif()
else()
    if(MSVC)
        target_compile_options(robotPathPlanner PRIVATE /Zi /Od /MDd)
    else()
        target_compile_options(robotPathPlanner PRIVATE -g -O0)
    endif()
endif()