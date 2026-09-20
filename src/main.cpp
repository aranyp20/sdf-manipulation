#include "Core/HessianBasedDeformer/HessianBasedDeformer.h"
#include "Representation/Csg.h"
#include "Representation/ImplicitBspline.h"
#include "Representation/Primitive.h"
#include "Visualization/Camera.h"
#include "Visualization/Renderer.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl2.h"

#include "imgui.h"

#include <GLFW/glfw3.h>
#include <cmath>

namespace {

struct AppState {
    Camera camera{Vec3::Zero(), 3.0};
    bool dirty = true;
    bool dragging = false;
    bool panning = false;
    double lastX = 0.0, lastY = 0.0;
    // Click = left press + release without moving; a drag (camera rotate) is not a click.
    double pressX = 0.0, pressY = 0.0;
    bool clickCandidate = false;
    bool click = false;
};

void OnMouseButton(GLFWwindow* window, int button, int action, int /*mods*/) {
    auto& state = *static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (ImGui::GetIO().WantCaptureMouse) {
        state.dragging = state.panning = false;
        state.clickCandidate = false;
        return;
    }
    const bool pressed = action == GLFW_PRESS;
    double x, y;
    glfwGetCursorPos(window, &x, &y);
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        state.dragging = pressed;
        if (pressed) {
            state.pressX = x;
            state.pressY = y;
            state.clickCandidate = true;
        } else if (state.clickCandidate) {
            constexpr double kClickSlopPx = 4.0;
            if (std::abs(x - state.pressX) < kClickSlopPx && std::abs(y - state.pressY) < kClickSlopPx) {
                state.click = true;
            }
            state.clickCandidate = false;
        }
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        state.panning = pressed;
    }
    state.lastX = x;
    state.lastY = y;
}

void OnCursorPos(GLFWwindow* window, double x, double y) {
    auto& state = *static_cast<AppState*>(glfwGetWindowUserPointer(window));
    const double dx = x - state.lastX;
    const double dy = y - state.lastY;
    state.lastX = x;
    state.lastY = y;
    if (state.dragging) {
        state.camera.Rotate(dx, dy);
        state.dirty = true;
    } else if (state.panning) {
        state.camera.Pan(dx, dy);
        state.dirty = true;
    }
}

void OnScroll(GLFWwindow* window, double /*dx*/, double dy) {
    auto& state = *static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (ImGui::GetIO().WantCaptureMouse) {
        return;
    }
    state.camera.Zoom(dy);
    state.dirty = true;
}

constexpr ImVec4 kMinimumColor = {0.2f, 0.9f, 0.2f, 1.0f}; // green
constexpr ImVec4 kOneSaddleColor = {0.2f, 0.4f, 1.0f, 1.0f}; // blue
constexpr ImVec4 kTwoSaddleColor = {1.0f, 0.9f, 0.1f, 1.0f}; // yellow
constexpr ImVec4 kMaximumColor = {1.0f, 0.2f, 0.2f, 1.0f}; // red
constexpr ImVec4 kSearchGridColor = {1.0f, 0.2f, 1.0f, 1.0f}; // magenta
constexpr ImVec4 kHighlightColor = {1.0f, 1.0f, 1.0f, 1.0f}; // hovered/selected: white

Vec3 ColorToVec3(const ImVec4& color) {
    return Vec3(color.x, color.y, color.z);
}

// Reference to one critical point: which list (negative eigenvalue count) and which entry.
struct CriticalPointRef {
    int type = -1;
    int index = -1;

    bool Valid() const {
        return type >= 0;
    }
};

const std::vector<Vec3>& CriticalPointList(const HBDebugData& data, int type) {
    const std::vector<Vec3>* lists[] = {&data.minimum, &data.oneSaddle, &data.twoSaddle, &data.maximum};
    return *lists[type];
}

// Nearest critical point within a few pixels of the cursor (GLFW coords, y down from the top).
CriticalPointRef PickCriticalPoint(const HBDebugData& data, const Camera& camera, int w, int h, double mouseX, double mouseY) {
    constexpr double kPickRadiusPx = 8.0;
    CriticalPointRef best;
    double bestDist2 = kPickRadiusPx * kPickRadiusPx;
    for (int type = 0; type < 4; ++type) {
        const std::vector<Vec3>& points = CriticalPointList(data, type);
        for (int i = 0; i < static_cast<int>(points.size()); ++i) {
            Vec2 ndc;
            if (!camera.ProjectToNdc(points[i], w, h, ndc)) {
                continue;
            }
            const double dx = (ndc.x() + 1.0) * 0.5 * w - mouseX;
            const double dy = (ndc.y() + 1.0) * 0.5 * h - (h - mouseY);
            const double dist2 = dx * dx + dy * dy;
            if (dist2 < bestDist2) {
                bestDist2 = dist2;
                best = {type, i};
            }
        }
    }
    return best;
}

void DrawCriticalPoints(const Renderer& renderer,
                        const HBDebugData& data,
                        const CriticalPointRef& hovered,
                        const CriticalPointRef& selected,
                        const Camera& camera,
                        int w,
                        int h) {
    renderer.DrawPoints(data.minimum, ColorToVec3(kMinimumColor), 8.0f, camera, w, h);
    renderer.DrawPoints(data.oneSaddle, ColorToVec3(kOneSaddleColor), 8.0f, camera, w, h);
    renderer.DrawPoints(data.twoSaddle, ColorToVec3(kTwoSaddleColor), 8.0f, camera, w, h);
    renderer.DrawPoints(data.maximum, ColorToVec3(kMaximumColor), 8.0f, camera, w, h);
    for (const CriticalPointRef& ref : {hovered, selected}) {
        if (ref.Valid()) {
            renderer.DrawPoints(
                {CriticalPointList(data, ref.type)[ref.index]}, ColorToVec3(kHighlightColor), 8.0f, camera, w, h);
        }
    }
}

void CriticalPointRow(const char* label, const std::vector<Vec3>& points, const ImVec4& color) {
    const float size = ImGui::GetFontSize();
    ImGui::ColorButton(label,
                       color,
                       ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoPicker | ImGuiColorEditFlags_NoDragDrop,
                       ImVec2(size, size));
    ImGui::SameLine();
    ImGui::Text("%s: %d", label, static_cast<int>(points.size()));
}

} // namespace

int main() {
    if (!glfwInit()) {
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(600, 600, "sdf_manipulation", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable | ImGuiConfigFlags_ViewportsEnable;

    // Install our callbacks before the ImGui backend's, so it chains to them.
    AppState state;
    glfwSetWindowUserPointer(window, &state);
    glfwSetMouseButtonCallback(window, OnMouseButton);
    glfwSetCursorPosCallback(window, OnCursorPos);
    glfwSetScrollCallback(window, OnScroll);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    Sphere sphere(Vec3::Zero(), 1.0);
    ImplicitBspline paca = ImplicitBspline::MakePaca();
    ImplicitBspline torus = ImplicitBspline::MakeTorus();
    Csg sphereCut = Csg::MakeSphereWithSphereCut();
    Csg cubeWithSphere = Csg::MakeCubeWithSphere();
    Csg halfPaca = Csg::MakeHalfPaca();
    struct ModelEntry {
        const char* name;
        const Implicit* model;
    };
    const ModelEntry models[] = {{"Sphere", &sphere},
                                 {"Paca", &paca},
                                 {"Torus", &torus},
                                 {"Sphere cut", &sphereCut},
                                 {"Cube with sphere", &cubeWithSphere},
                                 {"Half paca", &halfPaca}};
    int selectedModel = 1;

    HBDebugData hbDebugData;
    bool showSaddlePoints = true;
    bool showSearchGrid = false;
    bool showBoundingBox = false;
    bool showWorldAxes = false;
    CriticalPointRef selectedPoint;

    Renderer renderer;
    int lastW = 0, lastH = 0;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        int w, h, fbw, fbh;
        glfwGetWindowSize(window, &w, &h);
        glfwGetFramebufferSize(window, &fbw, &fbh);
        glViewport(0, 0, fbw, fbh);

        if (state.dirty || w != lastW || h != lastH) {
            renderer.ComputeNormalMap(*models[selectedModel].model, state.camera, w, h);
            renderer.ShadeDiffuse();
            state.dirty = false;
            lastW = w;
            lastH = h;
        }

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        renderer.Draw(fbw, fbh);

        const auto* selectedBspline = dynamic_cast<const ImplicitBspline*>(models[selectedModel].model);

        CriticalPointRef hoveredPoint;
        if (showSaddlePoints && selectedBspline && !ImGui::GetIO().WantCaptureMouse) {
            double mouseX, mouseY;
            glfwGetCursorPos(window, &mouseX, &mouseY);
            hoveredPoint = PickCriticalPoint(hbDebugData, state.camera, w, h, mouseX, mouseY);
        }
        if (state.click) {
            state.click = false;
            selectedPoint = hoveredPoint; // clicking empty space deselects
        }

        if (showWorldAxes) {
            renderer.DrawLine(Vec3::Zero(), Vec3::UnitX(), Vec3(1.0, 0.0, 0.0), state.camera, w, h);
            renderer.DrawLine(Vec3::Zero(), Vec3::UnitY(), Vec3(0.0, 1.0, 0.0), state.camera, w, h);
            renderer.DrawLine(Vec3::Zero(), Vec3::UnitZ(), Vec3(0.0, 0.0, 1.0), state.camera, w, h);
        }
        if (showBoundingBox) {
            renderer.DrawWireBox(
                models[selectedModel].model->GetBoundingBox(), ColorToVec3(kSearchGridColor), state.camera, w, h);
        }
        if (showSearchGrid && selectedBspline) {
            renderer.DrawWireBox(hbDebugData.searchBox, ColorToVec3(kSearchGridColor), state.camera, w, h);
        }
        if (showSaddlePoints && selectedBspline) {
            DrawCriticalPoints(renderer, hbDebugData, hoveredPoint, selectedPoint, state.camera, w, h);
        }

        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Models");
        for (int i = 0; i < static_cast<int>(std::size(models)); ++i) {
            if (ImGui::Selectable(models[i].name, selectedModel == i) && selectedModel != i) {
                selectedModel = i;
                selectedPoint = {};
                state.dirty = true;
            }
        }
        ImGui::End();

        ImGui::Begin("Common debug visu");
        ImGui::Checkbox("Show bounding box", &showBoundingBox);
        ImGui::Checkbox("Show world axes", &showWorldAxes);
        ImGui::End();

        ImGui::Begin("Hessian Deformer");
        if (ImGui::Button("Execute") && selectedBspline) {
            hbDebugData = HessianBasedDeformer(*selectedBspline).SearchSaddlePoints();
            selectedPoint = {};
        }
        ImGui::Checkbox("Show critical points", &showSaddlePoints);
        ImGui::Checkbox("Show critical grid", &showSearchGrid);
        CriticalPointRow("Minimum", hbDebugData.minimum, kMinimumColor);
        CriticalPointRow("1-saddle", hbDebugData.oneSaddle, kOneSaddleColor);
        CriticalPointRow("2-saddle", hbDebugData.twoSaddle, kTwoSaddleColor);
        CriticalPointRow("Maximum", hbDebugData.maximum, kMaximumColor);
        if (selectedPoint.Valid()) {
            const Vec3& p = CriticalPointList(hbDebugData, selectedPoint.type)[selectedPoint.index];
            ImGui::Text("(%.4f, %.4f, %.4f)", p.x(), p.y(), p.z());
        }
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            GLFWwindow* backup = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup);
        }

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
