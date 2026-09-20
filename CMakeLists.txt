cmake_minimum_required(VERSION 3.20)
project(sdf_manipulation CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# GLFW
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        docking
)
FetchContent_MakeAvailable(imgui)

# Eigen (header-only, skip its CMakeLists to avoid target conflicts)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    ON
)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
endif()
add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})

find_package(OpenGL REQUIRED)

set(IMGUI_DIR ${imgui_SOURCE_DIR})

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/Core/HessianBasedDeformer/HBDebugData.h
    src/Core/HessianBasedDeformer/HessianBasedDeformer.h
    src/Core/HessianBasedDeformer/HessianBasedDeformer.cpp
    src/Math/MyEigen.h
    src/Representation/Csg.h
    src/Representation/Csg.cpp
    src/Representation/Implicit.h
    src/Representation/ImplicitBspline.h
    src/Representation/ImplicitBspline.cpp
    src/Representation/Primitive.h
    src/Visualization/Camera.h
    src/Visualization/Camera.cpp
    src/Visualization/Renderer.h
    src/Visualization/Renderer.cpp
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl2.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    src
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    glfw
    OpenGL::GL
    Eigen3::Eigen
)

if(APPLE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE GL_SILENCE_DEPRECATION)
endif()
